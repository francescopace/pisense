# ADR: standardize managed CSI traffic sources

- Status: Accepted
- Date: 2026-08-23
- Updated: 2026-09-06

## Context

ESPectre needs a regular supply of HT20/HT-LTF packets so motion detection can cover its fixed temporal grid. Runtime diagnostics repeatedly showed average occupancy around 85% even when a traffic generator reported approximately `100 pps`. Raw callback rate alone could not distinguish missing packets from AP scheduling bursts, device-side send bursts, retransmissions, legacy-PHY delivery, same-slot excess, or processing backlog.

The project supports internal ICMP ping, internal DNS over UDP or TCP, external UDP delivered through the AP, and unicast ICMP Echo Requests supplied by an external host. These paths differ in protocol state, QoS treatment, target-specific Wi-Fi behavior, and missed-deadline behavior. The standalone external tool also initially lacked the fixed-phase behavior used by the internal generator.

Motion sensing also requires a precise definition of packet time. The CSI delivered by the Wi-Fi callback is estimated by the PHY from the packet training field at RF reception. Later callback or loop handling does not move that channel observation forward in time. The fixed admission grid must therefore use the Wi-Fi RX timestamp carried with the frame, while software clocks may measure only processing backlog.

## Evidence

### Capture method

Five monitor-mode captures were recorded on 2026-08-23 with a Mac on channel 2 (`2417 MHz`), an ESP32-C3 station at `ac:eb:e6:4a:e7:08`, and BSSID `e6:fa:c4:20:19:de`. Internal traffic crossed the gateway MAC `e4:fa:c4:b0:19:f8`; external traffic came from the Home Assistant host at `2c:cf:67:9b:80:2d`. The controlled target was `100 pps`.

The analysis decoded radiotap and IEEE 802.11 headers directly. It counted data-frame direction, Retry flags, QoS TID, HT versus legacy PHY, and gaps between observed non-Retry data frames. Control ACK, RTS, CTS, beacon, and unrelated data frames were excluded from the protocol tables. Because WPA3 payloads remained encrypted, protocol attribution comes from the controlled generator mode active during each capture rather than payload inspection.

The pcaps are local laboratory artifacts rather than versioned repository inputs. Their identities are retained here so an archived copy can be verified later.

| Mode | Local filename | Frames | Duration | SHA-256 prefix |
| --- | --- | ---: | ---: | --- |
| optimized ping | `espectre_air_ch2.pcap` | 31,286 | 57.159 s | `9bdc5a1bbff7` |
| C3 DNS/UDP | `espectre_air_dns_ch2.pcap` | 142,452 | 111.721 s | `31479d6ae622` |
| external UDP, DSCP 46 | `espectre_air_udp_external_ch2.pcap` | 68,568 | 189.611 s | `9b2f9035f343` |
| external UDP, best effort | `espectre_air_udp_be_ch2.pcap` | 100,112 | 291.025 s | `6f131aef9a0c` |
| DNS/TCP | `espectre_air_dns_tcp_ch2.pcap` | 86,860 | 155.657 s | `70450799fbca` |

### Generator-direction findings

`Retry share` is the fraction of observed data transmissions whose IEEE 802.11 Retry bit was set. It is not an AP retry counter: an independent sniffer can miss an original attempt or a retry. The large cross-mode differences remain useful, but the percentages are specific to this capture position and AP.

| Generator path | Measured direction | Observed data transmissions | Retry share | HT share | Dominant QoS mapping | Median non-Retry gap |
| --- | --- | ---: | ---: | ---: | --- | ---: |
| ping | C3 → AP | 6,080 | 2.81% | 99.74% | TID 5 / AC_VI | 10.009 ms |
| DNS/UDP | C3 → AP | 18,037 | 76.49% | 5.32% | TID 7 / AC_VO | 19.950 ms |
| DNS/TCP | C3 → AP | 17,760 | 4.05% | 99.78% | TID 5 / AC_VI | 9.941 ms |
| external UDP, DSCP 46 | AP → C3 | 22,188 | 9.73% | 99.93% | TID 6 / AC_VO | 9.978 ms |
| external UDP, best effort | AP → C3 | 33,621 | 9.29% | 99.93% | TID 3 / AC_BE | 9.988 ms |

DNS/UDP was the severe anomaly in the C3 matrix. Approximately 94.7% of its observed uplink transmissions used legacy PHY, predominantly `1` or `6 Mbit/s`, and approximately 94.7% carried TID 7. Its Retry-frame share was about twenty-seven times the optimized ping value. RTS/CTS exchanges and repeated attempts were visible around the legacy data frames. DNS/TCP removed this behavior on that target: almost every observed uplink frame returned to HT, its spacing returned to approximately `10 ms`, and its Retry share became close to ping.

DSCP 46 changed the external downlink mapping from this AP's TID 3 / AC_BE to predominantly TID 6 / AC_VO, but it did not materially reduce observed Retry share or median spacing on the otherwise healthy link. QoS marking is therefore a latency request, not a portable guarantee of higher occupancy. The exact TID selected for the same DSCP also differed by sender implementation: the C3 mapped its marked ping and DNS/TCP uplink mainly to TID 5 / AC_VI, while the AP mapped marked external downlink mainly to TID 6 / AC_VO.

### C3 receive-direction findings

The receive direction is the one that produces CSI on the C3. Every healthy path remained almost entirely HT in this direction.

| Mode | Observed AP → C3 Retry share | HT share | Median gap | 95th-percentile gap | Gaps below 5 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| ping | 7.62% | 99.86% | 9.976 ms | 15.845 ms | 12.59% |
| DNS/UDP | 9.92% | 99.93% | 9.465 ms | 21.512 ms | 21.82% |
| DNS/TCP | 8.46% | 99.91% | 9.951 ms | 15.076 ms | 9.84% |
| external UDP, DSCP 46 | 9.73% | 99.93% | 9.978 ms | 13.244 ms | 9.15% |
| external UDP, best effort | 9.29% | 99.93% | 9.988 ms | 13.500 ms | 9.42% |

The AP did not emit a perfectly uniform stream, but the paced healthy paths remained centered near `10 ms`. DNS/UDP had both the highest close-gap share and the longest 95th-percentile gap, which is consistent with its uplink retry behavior producing less regular downlink opportunities. The encrypted capture establishes correlation, not per-query causality. External DSCP and best-effort traffic were nearly indistinguishable in retry and spacing on this AP despite their different QoS access categories.

Short device-log observations agreed with the air captures. Optimized ping produced roughly 94% median occupancy. After the DNS/TCP firmware was flashed, the C3 reported `99-100 tx/s`, mostly `93-99%` steady-state occupancy with a median around 96%, and zero stale or out-of-order packets during the observed interval. External paced UDP reached approximately `96-98%` occupancy. These are diagnostic observations, not detector performance gates.

### Classic ESP32 DNS/UDP findings

Classic ESP32 investigation on 2026-08-29 showed that the C3 result was not portable across targets. Three 60-second Native runs with internal DNS/UDP at `100 pps` completed all `360/360` Direct attempts and all `180/180` diagnostic samples without censored failures, `csi:0/0`, generator errors, watchdogs, or reboots. Mean admitted occupancy ranged from `82.87%` to `86.42%` while the benchmark exercised the control plane.

A separate monitor-mode capture on channel 10 observed 70,178 frames over 118.492 seconds (`dns-udp-monitor-ch10-fixed.pcap`, SHA-256 prefix `2b2a04e3b248`). Classic ESP32 uplink remained almost entirely legacy `6 Mbit/s` with TID 7, but its Retry share was only `3.08%`, its median non-Retry gap was `10.022 ms`, and its non-Retry rate was `98.91 pps`. AP-to-device traffic was `99.97%` HT with a `2.63%` Retry share and a `9.998 ms` median gap. No radio gap exceeded `61.3 ms`; a post-capture diagnostic window reported `100.9` generated packets/s, `101.9` CSI callbacks/s, `94.9` admitted packets/s, and `95%` occupancy.

The classic capture preserves the legacy PHY, TID 7, and some RTS/CTS behavior that accompanied the C3 failure, but not its retry storm or cadence collapse. Legacy PHY and that WMM mapping are therefore exposure characteristics, not sufficient causes. Generator choice must remain device-, driver-, AP-, and resolver-specific.

### S3 internal ping QoS trial

A bounded A/B/A trial on 2026-09-06 compared `IP_TOS=184` (DSCP 46, Video), `IP_TOS=192` (DSCP 48, Voice), and a return to `184` on an ESP32-S3 at `10:b4:1d:e8:ec:00`, using ESP-IDF 5.5.5. The same socket, boot, gateway target, `100 pps` cadence, HT20 profile, and BSSID `e6:fa:c4:20:19:de` on channel 10 were retained. A passive ESP32-C6 observed that link while the Mac stayed associated. Temporary instrumentation reserved 112 KiB for software-send and hardware-RX records; it did not log individual packets during measurement.

Each setting had a 32-second S3 trace, with trace starts 210 seconds apart. Each trace contained 10 seconds of baseline HTTP observation, 10 seconds targeting 16 additional diagnostic GETs/s, and 10 seconds of recovery. Socket readback confirmed `184 → 192 → 184`. Regularity below means the percentage of consecutive first-reply hardware RX intervals within `8–12 ms`; it is distinct from admitted occupancy, whose mean uses the 30 diagnostic samples per setting.

| Setting | Successful sends | RX interval pairs | RX regularity | RX gaps below 5 ms | Mean occupancy |
| --- | ---: | ---: | ---: | ---: | ---: |
| Video A1 | 3,201 | 3,200 | 64.53% | 8.47% | 87.07% |
| Voice B | 3,200 | 3,197 | 65.31% | 7.79% | 92.37% |
| Video A2 | 3,200 | 3,199 | 64.55% | 8.78% | 91.07% |

The C6 confirmed uplink TID `5 → 6 → 5` during the scored windows. The corresponding replies used TID `6 → 7 → 6`; both reply TIDs belong to Voice. Consecutive uplink radio intervals pooled across the three HTTP windows were regular in `76.92%`, `79.05%`, and `75.87%` of 2,925, 2,926, and 2,864 pairs, respectively. Voice's observed uplink improvement was therefore 2.13–3.18 percentage points relative to the two Video intervals. Encrypted uplink identification used frame structure and sequence continuity; exact downlink matches to the S3 trace supplied the clock mapping. The radio capture was cropped to the scored windows because QoS changed after each frozen S3 dump, before the next measurement window.

C6 record checksums verified from a complete serial copy for each setting, with zero capture-buffer overflow. C6-to-S3 RX matching covered `97.18%`, `96.56%`, and `94.68%` of the S3 records. Requiring consecutive sequence numbers avoids treating missing observer frames as long packet intervals, but the differing coverage still limits comparisons; this is not a lossless radio capture.

All 9,601 sends succeeded, and 9,600 had a corresponding first CSI reply. The unmatched send in Voice does not establish packet loss on the network. All 475 actual load GETs and 90 diagnostic samples succeeded; the load scheduler issued 160, 157, and 158 requests against a target of 160 per setting. S3 trace checksums verified with zero record overflow or callback mismatch.

Voice improved observed RX regularity by approximately 0.8 percentage points relative to either Video interval, rather than the eight-point recovery hypothesized from a prior timing replay. This short run does not establish a repeatable gain. Occupancy remained elevated after returning to Video, so its increase cannot be attributed to the QoS setting. Frozen trace dumps paused the generator outside measurement and could shift its send phase relative to the unchanged admission grid. The replay's eliminated software-to-air variability also included medium-access waiting, rather than identifying a particular S3 queue.

Retain DSCP 46 as the production request. The trial does not justify changing the default to DSCP 48, and it does not establish a portable QoS improvement. Both original firmwares and all temporary production hooks were restored after acquisition.

The local evidence is under `espectre-s3-qos/run2`; `run1` was discarded because the HTTP load helper rejected an argument before load generation. Raw trace identities are retained for verification of an archived copy:

| Setting | S3 serial log SHA-256 prefix | C6 serial log SHA-256 prefix |
| --- | --- | --- |
| `a1-video` | `e888ab521ed5` | `571096c86cbd` |
| `b-voice` | `f9a792177796` | `5fdb99e641d3` |
| `a2-video` | `7d7a85822425` | `15b3c2a50db8` |

### S3 DNS/TCP and TX allocation trials

Two further bounded comparisons on 2026-09-06 retained the same S3, ESP-IDF version, BSSID, channel, gateway, HT20 profile, lightweight detector, and `100 pps` target. The protocol comparison used `ping → dns_tcp → ping` within one boot. The allocation comparison used fresh boots with `dynamic32 → static32 → dynamic32`; its effective configuration differences were limited to the Wi-Fi TX allocation selection, the corresponding 32-buffer count, and ESP-IDF's legacy aliases. Power saving, TX/RX AMPDU, CPU frequency, and task configuration were unchanged. The gateway at `192.168.1.1` answered a preliminary TCP/53 DNS query, so this test needed no manually supplied AP management address. MQTT was disconnected.

Each setting had 30 seconds of warm-up, a 17-second send trace, and two additional seconds of RX matching grace. Identical temporary arrays reserved 72 KiB, plus a bounded lifecycle-event buffer. No packet records were printed during acquisition. A passive C6 captured the same link for 22 seconds while the Mac remained associated. Each HTTP observation contained five seconds of baseline, five seconds targeting 16 additional diagnostic GETs/s, and five seconds of recovery, with one SSE connection throughout. Occupancy below averages the three trailing one-second snapshots wholly contained in each phase after a 100 ms boundary guard; the small sample count is not a detector performance gate. The 17-second RX window uses callback ESP timestamps for membership; regularity uses hardware RX timestamp intervals between eligible CSI records, including repeated replies, rather than counting all Wi-Fi frames or assuming every callback supplies sensing data.

| Protocol setting | Eligible CSI/s | RX intervals in 8–12 ms | Baseline occupancy | HTTP occupancy | Recovery occupancy |
| --- | ---: | ---: | ---: | ---: | ---: |
| Ping A1 | 100.06 | 59.29% | 84.67% | 78.00% | 84.00% |
| DNS/TCP | 98.12 | 55.13% | 97.00% | 91.67% | 98.00% |
| Ping A2 | 100.00 | 59.74% | 92.67% | 94.00% | 92.00% |

DNS/TCP reduced gaps below 5 ms from approximately 6.4% to 3.2%, but did not improve the 8–12 ms interval share. Its 95th-percentile gap improved to 14.81 ms from 15.51/15.47 ms, while its 99th percentile worsened to 18.85 ms from 17.03/16.81 ms. Its higher occupancy did not reverse when ping returned, and its HTTP occupancy was below the returning ping control. This again shows why ordered occupancy observations cannot isolate protocol effects from time and sender phase relative to the admission grid. The C6 also observed different complete behaviors: eligible DNS replies were 188-byte frames on downlink TID 3, while ping replies were 100-byte frames on TID 6. The production DNS eligibility filter rejects pure TCP ACKs and requires one complete DNS response per TCP segment; ACK traffic did not inflate the reported CSI supply. DNS/TCP remains an explicit compatibility-dependent option, with no demonstrated repeatable occupancy gain on this S3.

| TX buffers | Uplink intervals in 8–12 ms | RX intervals in 8–12 ms | Baseline occupancy | HTTP occupancy | Recovery occupancy |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dynamic32 A1 | 80.45% | 58.50% | 95.00% | 94.67% | 95.00% |
| Static32 B, qualified | 80.93% | 60.44% | 94.67% | 93.67% | 94.00% |
| Dynamic32 A2 | 79.87% | 59.80% | 94.33% | 94.33% | 94.00% |

The uplink comparison used 1,693, 1,699, and 1,694 adjacent MAC-sequence intervals within the common 17-second windows. It selected the observed 82-byte/TID-5 ping signature and excluded other HTTP frame signatures; encrypted payloads prevent direct ICMP identification. Exact C6-to-S3 downlink joins supplied the clock mapping, but observer omissions remain possible. Static allocation improved this observed uplink endpoint by only 0.48–1.06 percentage points, without improving occupancy. RX 95th percentiles were 15.19/15.28/15.35 ms and 99th percentiles were 16.65/16.61/16.68 ms; gaps of at least 20 ms occurred 1/2/1 times. These short, qualified measurements do not establish a useful gain from static allocation.

Static32 reduced observed free heap after capture by 55,448–55,500 bytes, approximately 54.2 KiB, relative to its dynamic controls. The nominal 32 × 1,600-byte static TX payload reservation is 50 KiB; the whole observed difference is not an isolated allocation measurement. The largest free internal DMA block was approximately 38 KiB with static allocation versus 92 KiB with dynamic allocation. These are snapshots with the same temporary trace memory present, not measurements of peak DMA demand. Retain dynamic32; the result does not justify reserving additional memory or identify dynamic TX allocation as the source of the remaining timing variation.

Serial-export integrity limits were retained explicitly. The DNS run lost 23 TX log rows, while all 1,862 RX records and their checksum verified; its results therefore use RX-only timing and independent HTTP telemetry, without a query success-rate claim. The first static run lost 21 RX rows, including primary-window data, and was excluded from the timing comparison. A repeat slowed only the frozen export to two ticks per row; it still lost three RX rows, all in the grace interval after the verified TX ID range 3,001–4,700. Every one of those 1,700 primary queries had a recorded first reply, and independent C6 joins supported the common timing window, but the complete S3 RX checksum still failed. The repeated static timing is consequently qualified, not a fully checksum-verified capture. The flanking dynamic traces passed all component checksums. Missing serial rows are not evidence of packet loss on the network.

All six tabled observations completed their 478/478 actual load GETs and 90/90 diagnostic samples. Their C6 record checksums verified with no capture overflow, and the firmware reported no trace overflow or callback-identity mismatch. Both original firmwares were restored after acquisition. The restored S3 application rebuilt byte-for-byte to SHA-256 `419c50513be7…`, with its original dynamic32 configuration, and returned ready with `ping`, `100 pps`, and the pinned BSSID. All temporary production hooks were removed.

Local evidence is under `espectre-s3-followup`, including component validation, the qualified static analysis, independent uplink CSVs, effective configuration comparisons, artifact manifests, and restoration checks. Raw serial identities are retained for verification of an archived copy:

| Observation | S3 serial log SHA-256 prefix | C6 serial log SHA-256 prefix |
| --- | --- | --- |
| `run1/p1-ping-dynamic` | `64d61c61de5e` | `86187d4b5ee9` |
| `run1/p2-dns-tcp-dynamic` | `d709ba0e60cb` | `b175e27bfffd` |
| `run1/p3-ping-dynamic` | `530d4a080a7d` | `8859e21e40d2` |
| `run1/p4-ping-static`, excluded timing | `f70fe5a6c0c4` | `69bdef147a94` |
| `run1/p5-ping-dynamic` | `8b824b474761` | `19f02bdba1d0` |
| `run2/r1-ping-static`, qualified timing | `1a2a98802150` | `1a622886f53c` |
| `run2/r2-ping-dynamic` | `2eb08092a3b4` | `4274bdbbd11b` |

### S3 driver handoff and sensing bypass trial

A further bounded A/B/A trial on 2026-09-06 compared the normal sensing path, a minimal CSI callback, and the restored normal path within one S3 boot and ping socket. It retained the same BSSID, channel, gateway, HT20 profile, lightweight detector, `100 pps`, dynamic32 allocation, and 160 MHz configuration. Each treatment had 30 seconds of settling, a 17-second send window, and two seconds of RX grace. The temporary probe recorded application `sendto` entry/return, `esp_netif_transmit_wrap` entry/return, and identified CSI reply timestamps in 95 KiB of fixed arrays. Export ran only after capture was frozen, on a temporary task that was deleted before the next measurement. The C6 passively captured the link, target and foreign beacons, sampled surrounding headers, and 10 ms received-frame count bins.

The minimal treatment retained the common trace parser but bypassed production CSI processing, queueing, and resulting inference. It did not disable CSI generation or processing inside the Wi-Fi driver. Each treatment used five seconds of baseline, five seconds targeting 16 additional diagnostic GETs/s, and five seconds of recovery. SSE was disabled in every treatment so that suppressing detector events could not remove an extra stream of radio traffic. Occupancy during the bypass is not a valid sensing endpoint.

All 5,100 sends had an exact driver-handoff record and a first reply, with no send or handoff error. The application-to-handoff median was 186/182/187 µs, its 95th percentile was 309/311/291 µs, and its maximum was 1,097/1,275/1,131 µs. No such wait exceeded 2 ms. The wrapper-call 95th percentile was 661/492/662 µs. Interval regularity below means `[8, 12) ms`; software and S3 RX columns use the complete 17-second windows, while the radio comparison uses the well-observed baseline windows.

| Treatment | APP intervals regular | Handoff intervals regular | Mean callback duration | Baseline uplink intervals regular | S3 RX intervals regular |
| --- | ---: | ---: | ---: | ---: | ---: |
| Normal A1 | 99.82% | 99.76% | 224.0 µs | 78.20% | 63.74% |
| Minimal B | 99.76% | 99.65% | 42.0 µs | 78.74% | 63.92% |
| Normal A2 | 99.94% | 99.88% | 221.7 µs | 79.45% | 66.33% |

The baseline radio comparison contains 477/475/477 adjacent MAC-sequence pairs, corresponding to 100%/99.58%/100% of the expected adjacent application pairs in those scored windows. Uplink identification remains conditional on the encrypted 82-byte/TID-5 signature and sequence continuity. Exact downlink joins separately validate the C6-to-S3 hardware clock mapping. The minimal callback reduced measured callback time substantially without demonstrating an uplink or RX regularity benefit; callback durations include preemption and are not isolated CPU-time measurements. Under HTTP, complete S3 RX regularity was 65.05%/63.87%/66.46%, also without a bypass benefit. This result does not establish equivalence across other chips, detectors, or workloads.

The measured pre-driver path does not explain most of the observed radio timing variation. In this ESP-IDF configuration, `sendto` waits for TCPIP processing, and the wrapped netif call forwards into `esp_wifi_internal_tx`; successful return establishes driver acceptance, not transmission over the air. Subsequent variation can still arise in driver/MAC scheduling or medium access. Radio transmission can begin before the wrapped call returns, and C6, S3 hardware, and S3 software timestamps belong to different clocks. Conditional latency bounds therefore retain their clock assumptions; no exact absolute queue residence time is claimed.

The pinned BSSID directly transmitted beacons advertising a 100-TU (102.4 ms) interval, consistent with the earlier periodicity observation; actual arrivals can be delayed. In the two normal controls, target beacons were 2.92 and 1.55 times as concentrated inside long uplink intervals (at least 12 ms) as their share of observed elapsed time would predict; the later nine-second holdouts retained enrichment of 2.86 and 1.75. This supports an association with beacon activity, but its variable strength and multiple staggered beacon sources do not establish that the target AP alone causes the delays. A four-harmonic 102.4 ms model of conditional handoff-to-air variation had held-out `R²` of -0.015 in A1 and 0.149 in A2; the earlier periodicity observation must not be promoted to a universal driver-delay model.

Observer coverage is a separate limit from trace integrity. In B, the C6 recorded no frames in 466 ten-millisecond bins (4.66 seconds total), including two continuous gaps of 1.60 and 2.74 seconds. Its HTTP and recovery uplink pair coverage fell to 77.94% and 31.24%, so those radio subsets are excluded from treatment-effect conclusions. The S3 still recorded every first reply. Foreign-beacon pools overflowed by 73/0/124 records; target-beacon, link, and context pools did not overflow. Valid beacon fixed fields and TIM data were retained even when the trailing IE parse was incomplete; this does not establish malformed AP transmissions. Received-frame bins and sampled headers are not CCA-busy or airtime measurements and cannot exclude hidden contenders.

All decoded component checksums verified. Four APP records were absent from the first serial copy of A1, but both later copies were complete and verified; no missing records were invented or inferred. All other S3 streams and all C6 pools had complete verified copies. The 237/237 actual load GETs and 45/45 diagnostic samples succeeded, with no SSE connection. A preliminary run was discarded after the probe overflowed the generator stack during frozen export; moving export to its own task fixed that instrumentation fault without increasing the measured generator stack. The repeated run retained at least 5,348 bytes of export-task stack headroom.

Retain the production sensing path and existing task and buffer configuration. This experiment does not justify a scheduling or queue change for occupancy. All probe changes were isolated from production sources and the normal build. Both original full-flash images were restored and verified; the S3 returned ready with its original application SHA-256 `419c50513be7…`, `ping`, `100 pps`, and the pinned BSSID. Local evidence is under `espectre-s3-outbound/run2`, with beacon analysis under `espectre-s3-outbound/beacon-association` and independent restoration verification.

| Observation | S3 serial log SHA-256 prefix | C6 serial log SHA-256 prefix |
| --- | --- | --- |
| `run2/p1-normal` | `f2965d3e66c4` | `0e14d9249acd` |
| `run2/p2-minimal` | `c01a5ae8cc65` | `18111caa5bb6` |
| `run2/p3-normal` | `d6997a5254dd` | `b6428954b35f` |

### S3 CSI-disabled isolation trial

A subsequent same-boot A/B/A trial on 2026-09-06 retained the preceding S3 configuration and used one ping socket throughout A1/B/A2, but called `esp_wifi_set_csi(false)` for B and re-enabled CSI for A2. This disables the exposed CSI feature rather than merely bypassing the application callback; it does not disable ordinary PHY channel estimation needed to receive Wi-Fi. The public API wrapper recorded successful transitions, no unexpected calls, and no API errors. B produced zero callbacks and zero CSI RX records throughout the 17-second capture and two-second grace period. The runtime service's logical enabled flag was not used as evidence of hardware state. The trace allocation, 30-second settling period, five-second baseline/load/recovery phases, 16 additional HTTP GETs/s, and absence of SSE remained unchanged.

The C6 probe added a callback-entry `esp_timer` timestamp to each link record and bracketed every capture with six serial clock exchanges before and after acquisition. Feasible affine clock mappings retained the complete host send/receive brackets; cohort boundaries had 9.78–11.48 ms uncertainty and an additional 100 ms guard. Radio intervals still used C6 hardware RX timestamps. This allowed an independently timed OFF cohort without reusing an ON-only S3 CSI clock fit. Uplink and downlink separately required at least 98% packet-count and adjacent-pair coverage against the actual application cohort. Encrypted frame identity remains conditional on the link, TID, length, and sequence signature. Exact CSI joins validated the downlink signature independently in each ON control; OFF downlink is an observed AP-to-S3 cadence proxy, not proof of individual ping delivery.

All 5,100 sends succeeded. There were 5,099 matching driver-handoff records: A1's final send started 185 µs before the capture cutoff and returned afterward, making its absent handoff compatible with boundary censoring. It fell outside every scored HTTP cohort. Both ON controls recorded all 1,700 first replies; OFF deliberately had no CSI-based reply observation. Application-to-handoff 95th percentiles were 316/314/304 µs, with maxima of 1,335/1,191/1,132 µs. Wrapper-call 95th percentiles were 684/502/683 µs, and mean measured callback duration was 240.4/0/242.7 µs. Callback durations include preemption. Regularity below means `[8, 12) ms`; APP, handoff, and S3 RX use complete 17-second windows, while C6 radio columns use the qualified baseline cohorts.

| Treatment | APP intervals regular | Handoff intervals regular | Baseline uplink intervals regular | Baseline downlink intervals regular | S3 RX intervals regular |
| --- | ---: | ---: | ---: | ---: | ---: |
| CSI on A1 | 99.71% | 99.47% | 80.96% | 61.34% | 62.39% |
| CSI off B | 99.88% | 99.94% | 85.12% | 71.00% | Unavailable |
| CSI on A2 | 99.88% | 99.71% | 84.66% | 70.46% | 70.57% |

The baseline uplink comparison contains 478/477/476 adjacent pairs, with count coverage of 100.21%/100%/99.79%; the corresponding downlink counts are 476/469/474, with 99.79%/98.32%/99.37% coverage. Ratios slightly above 100% reflect independently mapped, guarded cohort boundaries and conditional radio identity, rather than delivery probabilities. The apparent A1-to-B improvement remained after CSI was re-enabled. The independently complete S3 RX trace also improved between the two ON controls, including 62.47% to 69.33% regularity under HTTP. This does not demonstrate a reversible CSI-disable benefit or prove zero CSI influence: time variation, sender phase, and possible toggle carryover remain alternatives. Even with CSI disabled, substantial irregularity remained between the regular driver handoff and observed radio transmission.

The third C6 capture cannot support an under-load A/B/A conclusion. Its HTTP/recovery uplink pair coverage fell to 83.82%/13.00%, and downlink coverage to 78.57%/12.37%; both subsets are excluded. It contained all-frame observation gaps of 0.95 and 3.45 seconds while the S3 still recorded every first reply. C6 callback-minus-hardware variation also reached approximately 123 ms under load and 1.10 seconds in recovery, exceeding the cohort guard. The qualified A2 baseline had approximately 10.4 ms of this variation and no empty all-frame bins in its scored envelope. The target beacon's advertised and observed nominal cadence remained 100 TU, but baseline target-beacon enrichment inside long uplink intervals varied from 1.45 to 0.58 to zero across A1/B/A2. These observations do not identify a stable target-beacon cause, and the sniffer does not measure medium-busy time.

All frozen component checksums verified, all 240 actual load GETs and 45 diagnostic samples succeeded, and S3 trace, C6 link, target-beacon, and context pools had no overflow. Foreign-beacon pools overflowed by 75/79/0 records and cannot describe complete neighboring activity. Every export retained at least 5,352 bytes of task-stack headroom. A preliminary run was excluded because delayed HTTP startup placed scored phases outside the S3 trace; a post-capture clock command could also be discarded before the C6 returned ready. The repeated acquisition imported the HTTP client before resetting the S3, checked phase containment, and waited for C6 readiness before requesting post-capture anchors.

Retain the production CSI path and current task and buffer configuration. The measured application-to-driver path does not explain most radio interval variation, but these traces cannot separate internal driver/MAC scheduling from channel access, contention, or retries. A further completion timestamp inside the S3 could measure driver handoff to software completion on the same clock, after validating packet identity and callback timing. That interval would still include medium access, retries, and callback-dispatch delay; it would not isolate queue residence or provide a hardware RF or ACK timestamp. All probe changes remained outside production sources and builds. At the user's request, the lab firmware was left installed on both devices, with the C6 idle and the S3 ready, CSI re-enabled, the pinned BSSID unchanged, and `ping` targeting `100 pps`. A final live check observed 100 generated and accepted packets/s with zero pending-frame drops; its 95% occupancy is an operational snapshot, not a treatment-effect result.

Local evidence is under `espectre-s3-csi-off/run2`, including clock anchors, decoded traces, independent beacon audits, and live verification. The S3 probe application SHA-256 is `ed98e87da95a…`, and the C6 probe application SHA-256 is `ea258f9de01c…`.

| Observation | S3 serial log SHA-256 prefix | C6 serial log SHA-256 prefix |
| --- | --- | --- |
| `run2/p1-normal` | `30a52b968fa6` | `155ac15318a7` |
| `run2/p2-csi_off` | `317228fcd2f7` | `c9a6177d6dfb` |
| `run2/p3-normal` | `ec471e052f9f` | `e663859906c7` |

### S3 TX-completion timing trial

A subsequent isolation probe used the ESP-IDF 5.5.5 private `esp_wifi_set_tx_done_cb` API to observe software transmission completion on the same S3 clock as application sends and driver handoffs. The installed header describes a transmitted-data pointer, a length pointer, an interface index, and a success boolean; it does not specify a hardware RF timestamp or establish independently observed ACK delivery. The temporary callback retained bounded values only, without modifying or retaining driver-owned buffers, allocating memory, or logging during capture. Registration and CSI state were audited, admitted writers were drained before export, and the completion callback was unregistered before the frozen records were printed.

An initial five-second pilot, following 30 seconds of settling and including one second of completion grace, recorded 500 application sends, 501 handoffs, and 600 completion callbacks with CSI enabled. Every application send had one matching handoff; the extra handoff belonged to a send crossing the opening boundary. Application-to-handoff latency had a median of 192 µs, a 95th percentile of 292 µs, and a maximum of 522 µs. All completion callbacks reported success and ran in task context on core 0. The bounded prefix-copy instrumentation took 32 µs at the median, 51 µs at the 95th percentile, and 83 µs at the maximum; these elapsed times include possible preemption.

The pilot established a necessary packet-format distinction. Every callback reported 56 accessible bytes containing a protected QoS Wi-Fi header, a structural eight-byte security header, plaintext LLC/SNAP at offset 34, and only the first 14 bytes of an IPv4 header at offset 42. The ICMP identifier and sequence were outside the reported length and were not read. IPv4 identification, MAC sequence control, and a six-byte security-header PN candidate were visible and individually unique across all 600 records. The observed link addresses, TID 5, IPv4 version, 20-byte IPv4 header length, 28-byte IP total length, and ICMP protocol were consistent throughout. These observations motivated recording IPv4 identification at the driver handoff and validating the Wi-Fi header identifiers against the independent C6 capture before joining timing records.

All pilot component checksums verified: APP `d4383234`, HANDOFF `755e5cca`, and DONE `5ba16d49`. Two APP serial copies and all three HANDOFF/DONE copies were complete; the incomplete APP copy was not filled with invented records. Fifteen sanitizer cases exercised the actual callback body, including bounded copies, null pointers, capacity overflow, and concurrent publication. Local pilot evidence is under `espectre-s3-txdone/pilot1`; the pilot application SHA-256 is `4fc21abc9cd83…`.

The full probe retained CSI, the existing firmware configuration, and the generator stack. It captured 17 seconds of APP/HAND records and two additional seconds of RX/DONE grace after 30 seconds of settling. Fixed arrays occupied 153,600 bytes: APP16, HAND24, RX16, and DONE24 records. The completion recognizer accepted only the pilot-validated bounded header layout, with explicit rejection counters. The C6 was reset before acquisition and used only its first capture after boot; this was an observer-lifecycle mitigation rather than a demonstrated repair of its earlier deterioration. HTTP again used five-second baseline/load/recovery phases, targeting 16 additional GETs/s without SSE. Serial clock brackets and guarded cohort containment were retained.

All 1,700 application sends linked uniquely to their driver handoff by ICMP identifier/sequence, then to completion by the independently recorded IPv4 identification, and finally to C6 uplink frames by the pinned link, TID, key ID, PN, and raw MAC sequence control. No ordinal matching or key collision was involved. All 1,699 adjacent application-query pairs were observed in every uplink timing stream. The complete grace window contained 1,900 DONE identities and 1,951 C6 attempts matching those identities. Within the primary window there were 42 extra observed attempts and eight packet groups whose first observed attempt already had Retry set. First and last observed attempts are therefore distinct endpoints, and neither label guarantees that the observer saw every physical attempt. Callback length 56 was not equated with the observed radio frame length of 82 bytes.

| Measurement boundary | Intervals in `[8, 12) ms`, same 1,699 query pairs |
| --- | ---: |
| Application send entry | 100% |
| Driver handoff entry | 99.76% |
| First observed radio attempt | 74.04% |
| Last observed radio attempt | 73.28% |
| TX-completion callback entry | 73.16% |

On the single S3 software clock, application-to-handoff latency had a 197 µs median, 315 µs 95th percentile, and 2,504 µs maximum. Handoff-to-completion latency had a 693 µs median, 4,411 µs 95th percentile, 8,665 µs 99th percentile, and 14,739 µs maximum. Five completion callbacks preceded the wrapped call's return, and 47 preceded `sendto` return; these signed observations were retained. A return timestamp is consequently unsuitable as an assumed boundary preceding physical transmission. All 1,700 sends and handoff calls succeeded, all 1,900 completion records reported success in task context on core 0, and all 1,700 first replies were present in the independent S3 CSI trace.

Completion intervals and last-observed-radio intervals correlated at 0.99924; the 95th percentile of their absolute interval difference was 157 µs. This is interval agreement between exactly identified packets, not an absolute completion-to-air latency measurement. It places most observed cadence variation after driver handoff and before or at radio transmission, rather than attributing it solely to delayed completion notification. Of 280 packets with handoff-to-completion delay of at least 2 ms, 247 had only one observed non-Retry attempt; 33 had an observed Retry indication. Of 11 delays of at least 10 ms, six had an observed Retry indication. Observed retries do not explain most of the 2 ms cases, but missing attempts, initial contention/backoff, and internal driver/MAC work remain possible.

The guarded baseline/load/recovery cohorts contained 477/476/477 adjacent query pairs. First-observed-radio regularity was 74.42%/75.21%/74.42%, last-observed-radio regularity was 74.21%/74.37%/72.54%, and completion regularity was 73.79%/74.16%/72.54%. Handoff-to-completion 95th percentiles were 4.61/4.17/4.68 ms. This bounded HTTP workload did not produce a distinct outbound degradation in this trace; the ordered short windows do not establish equivalence under other loads or across boots. The actual workload issued 79 successful load GETs, corresponding to 15.8/s, and all 15 diagnostic samples succeeded.

A fresh C6-to-S3 hardware RX clock fit used 1,894 unique downlink matches, including a 378-pair holdout, with approximately ±1.03 µs residual and -6.105 ppm relative scale. Independent clock bounds used handoff-before-uplink and downlink-before-CSI-callback constraints; they did not assume completion occurs after the radio observation. Allowing software/hardware clock drift and a 100 µs timing margin left many individual completion-versus-air orderings unresolved. The interval agreement above is the stronger supported result. No pure queue-residence time or hardware TX/ACK timestamp is inferred from the completion callback.

All S3/C6 component checksums verified. C6 had no empty ten-millisecond receive bins, complete target-beacon cadence, no link or target-beacon overflow, and approximately 1.21 ms of callback-minus-hardware variation. Foreign-beacon storage overflowed by 31 records and does not describe complete neighboring activity. S3 free heap was 59,592 bytes at arm and 58,776 bytes at freeze, with a 31,744-byte largest internal-DMA block at both points. The full completion probe's measured callback duration had a 52 µs median, approximately 98 µs 95th percentile, and 151 µs maximum, excluding the final admission-counter release. The export worker retained at least 5,280 bytes of stack headroom. A further 699 sanitizer cases validated the full bounded recognizer and callback, including all 600 actual pilot headers. These figures document instrumentation cost; this is a diagnostic capture rather than a production performance benchmark.

Retain the production task, buffer, and pacing settings. The experiment now identifies the same outgoing packets across software and radio observations and quantifies a variable interval after driver handoff. It does not separate internal Wi-Fi scheduling from waiting for the shared channel, so it does not yet justify a specific firmware optimization for occupancy. The diagnostic firmware remains installed, with CSI active and the completion callback unregistered after capture. Final operational checks showed ready sensing, approximately 100 generated and accepted packets/s, and zero pending-frame drops. Local evidence is under `espectre-s3-txdone/full1/p1-normal`; application SHA-256 is `59f1333a8eb5…`, S3 serial SHA-256 is `188b98f053ee…`, and C6 serial SHA-256 is `22bd031da821…`.

### S3 AP and channel control trial

A subsequent controlled comparison used the current AP's self address (`192.168.1.249`, BSSID `e6:fa:c4:20:19:de`) and an isolated ESP32-D0WD SoftAP (`192.168.77.1`, BSSID `4c:11:ae:b8:6e:b9`). The planned sequence was original AP/channel 10, temporary AP/channel 10, temporary AP/channel 1, temporary AP/channel 10, and original AP/channel 10. Both destinations were AP-local, removing gateway forwarding from this comparison. The temporary AP used WPA2-PSK/CCMP, HT20, 100-TU beacons, one permitted station, and no upstream connection. S3 association logs in both A1 and A2 explicitly report `WPA3-SAE H2E`; AP metadata records CCMP. The AP comparison therefore changed security, peer implementation, and association behavior as well as AP identity.

The S3 retained the same native lightweight firmware, CSI, 100 pps, task placement, buffering, and pacing throughout. Temporary association and destination controls also updated the CSI source filter, which otherwise expects gateway replies. Each condition waited at least 30 seconds after stable association before recording 17 seconds of APP/HAND and two additional seconds of RX/DONE grace. No HTTP workload was deliberately added; preflight showed MQTT unconfigured and no Direct HTTP event clients. The temporary AP was stopped during both original-AP measurements. A passive survey selected channel 1 after observing 0 and 6 valid frames in two five-second samples, versus 2,758 in one channel-10 sample; this measured decoded activity, not CCA or undecodable interference.

The C6 took its first capture after a validated application-level reboot for every condition. Its initial channel-1 recording exhausted the 5,500-record link pool: 6,019 attempts were counted, with heavy downlink retries already consuming space before the scored window. Fifty primary uplinks were unmatched, including 49 after storage exhaustion and one earlier omission. A separately built observer variant increased the link pool to 7,000 while reducing foreign-beacon and sampled-context pools, lowering total fixed storage from 264,368 to 261,312 bytes. The callback implementation, record layouts, and S3 firmware remained unchanged. An additional C2 observation used this variant after B2 and before A2; the original incomplete C observation is retained below. A2 also used the expanded observer. Runtime allocation readback, schema hashes, build artifacts, and flash verification distinguish both observer versions.

All six software traces contain 1,700 unique APP/HAND/DONE links and 1,699 consecutive query pairs. Application cadence was at least 99.94% regular, and driver-handoff cadence was at least 99.94% regular. The table defines regularity as intervals in `[8, 12) ms`; it is not detector occupancy. Completion values use the entire S3 trace even where radio coverage is incomplete. Radio percentages are shown only for complete exact primary packet coverage.

| Condition, acquisition order | Regular completion intervals | Handoff-to-completion p95 | Regular first-observed radio intervals | Exact primary radio coverage |
| --- | ---: | ---: | ---: | ---: |
| A1: original AP, channel 10 | 83.28% | 3.643 ms | 83.70% | 1,700/1,700 |
| B1: temporary AP, channel 10 | 80.93% | 3.659 ms | 81.28% | 1,700/1,700 |
| C: temporary AP, channel 1 | 98.18% | 2.214 ms | Incomplete | 1,650/1,700 |
| B2: temporary AP, channel 10 | 74.87% | 4.401 ms | Incomplete | 1,698/1,700 |
| C2: corrective repeat, channel 1 | 96.00% | 2.373 ms | 97.35% | 1,700/1,700 |
| A2: original AP, channel 10 | 78.81% | 5.699 ms | Incomplete | 1,699/1,700 |

The channel-1 improvement appeared twice and reversed on the intervening channel-10 return. The complete C2 radio observation confirms that this was not merely a change in completion-notification timing. First observed uplink attempts among exact joins remained predominantly HT20/MCS7; C2's better cadence coexisted with substantially more observed retries than B2, so retry count alone does not explain the difference. However, S3 RSSI and guard-interval/rate behavior changed across associations, and these short, ordered traces are not randomized independent trials. Captured beacons do not include WMM/EDCA parameters, and received-frame bins do not measure medium contention. The result supports sensitivity to the channel and connection context after driver handoff; it does not separate internal Wi-Fi scheduling from access to the shared medium or establish a pure queue-residence time.

Incoming sensing supply remains a separate limitation. Unique eligible first CSI replies were A1 1,700/1,700, B1 1,245/1,700, C 1,249/1,700, B2 1,700/1,700, C2 1,065/1,700, and A2 1,700/1,700. Missing eligible CSI replies are not proof of AP packet loss: the temporary AP's observed downlink included legacy frames, changing HT rates, and retries, while encrypted radio headers cannot assign every non-CSI frame to a missing ICMP reply. Consequently, more regular outgoing traffic in this experiment does not establish an occupancy improvement.

Strict stream layout and checksum validation passed for all acquisitions. Incomplete redundant serial copies were retained as export warnings; intact copies provided the original complete software streams. B2's two missing radio identities, A2's one missing identity, and C's overflow remain explicit radio-quality limitations. C2 had complete primary identities, no primary-pool overflow, no empty receive bins over the primary radio envelope, and 97 microseconds of callback-minus-hardware variation; two approximately 205 ms target-beacon gaps and sampled-context overflow remain recorded limitations. AP health showed no association-counter change within scored windows, and independent audits found no S3 reset, reassociation, or generator restart after arming.

Keep production task, queue, and pacing settings unchanged. The comparison identifies a repeatable outgoing timing difference without changing S3 software, but it does not provide a demonstrated firmware optimization under an uncontrollable AP. Final checks leave the S3 on the original pinned AP with CSI and ready sensing active, the completion callback unregistered, the C6 idle, and the temporary AP stopped. Local evidence is under `espectre-s3-ap-control/run1`, including the chronological comparison, independent audits, and final manifest. S3 application SHA-256 is `97958342aca0…`; temporary AP is `0de62e7aa45e…`; C6 is `35f2fdbd1763…` for A1/B1/C/B2 and `b404b673bd58…` for C2/A2. No production firmware sources were changed by this experiment.

### S3 internal transmit instrumentation

A 2026-09-07 experiment instrumented the S3's ESP-IDF 5.5.5 transmit path in an isolated copy of the preceding native lightweight test firmware. The original AP remained pinned to `e6:fa:c4:20:19:de`, channel 10, with AP-local destination `192.168.1.249`, CSI enabled, and 100 pps. No HTTP workload was added. The frozen application hash is `f73ecb289fe18bc03edfe08d8ab228adf6e549cb89736ac4628d89083947068c`; effective SDK configuration is unchanged from the preceding AP/channel experiment. These private-symbol hooks are specific to the inspected S3 libraries and are not a production or portable driver API.

Linker wrappers recorded `ppTxPkt`, `lmacTxFrame`, and `hal_mac_tx_set_ppdu` entry/return events in a fixed 32 KiB internal-RAM buffer. Recording code, helper functions, and wrapper entry points were verified in IRAM; timestamps and bounded record writes occur without hot-path logging or allocation. The attempted `ppDequeueTxQ` wrapper was not traversed, so the experiment does not measure pure frame-queue residence or OSI event-queue latency. An initial pilot also showed that the handoff and `ppTxPkt` run in different tasks. Its proposed same-task identity association was rejected. The corrected probe instead validates the protected QoS/IPv4 header through the descriptor's buffer pointer, using the pointer construction found in this SDK's `ppProcTxDone`, and records the IPv4 ID at LMAC entry. The existing unique HAND/DONE IDs and DONE sequence/PN-to-C6 match then establish packet identity without a nearest-timestamp association.

The corrected comparison used the same binary with recording enabled, disabled, and enabled again. Every run captured 17 seconds of APP/HAND and two seconds of RX/DONE grace, with 1,700 exact APP/HAND/DONE matches; the raw RX and DONE streams contain 1,900 records because traffic continues during grace. Internal recording covers only the first second of each enabled run. Both internal windows contain 100 uniquely identified ping descriptors and complete exact C6 first-attempt observations, with no duplicate stages, premature descriptor reuse, or internal/primary-radio pool overflow. Each window also contains one rejected additional driver frame. All three redundant internal exports were complete and checksum-valid.

| Internal one-second window | Handoff to LMAC return, p95 | `ppTxPkt` entry to LMAC entry, p95 | LMAC return to TX callback, p95 / maximum | Regular LMAC-return intervals | Regular first-observed radio intervals |
| --- | ---: | ---: | ---: | ---: | ---: |
| Recording enabled, first run | 0.577 ms | 0.121 ms | 5.453 / 13.389 ms | 97.98% | 70.71% |
| Recording enabled, repeat | 0.507 ms | 0.114 ms | 3.489 / 9.350 ms | 100.00% | 79.80% |

Regularity again means intervals in `[8, 12) ms`, with 99 consecutive pairs per internal window, rather than sensing occupancy. Handoff and `ppTxPkt` entry intervals were 100% regular in both internal windows. Changes in the post-LMAC completion delay closely follow changes in the independently observed last-radio delay: interval-based correlations are 0.99920 and 0.99947. No arbitrary offset between the S3 and C6 clocks is subtracted. Each internal window has one observed retransmission, and no primary identity starts with an observed retry. The first-attempt cadence itself is already irregular. The first run's isolated 2.666 ms `ppTxPkt`-to-LMAC wait follows a packet whose post-LMAC completion took 13.389 ms; this is consistent with backlog behind a slow preceding transmission, rather than establishing a separate periodic software stall.

With recording disabled, the full trace still has only 76.63% regular completion intervals and 77.58% regular first-radio intervals. The enabled repeat has 77.93% regular completion intervals over its full 17 seconds, versus 69.04% in the first enabled run. Median handoff-call duration is 439/427/440 microseconds in acquisition order. This short, ordered comparison supports persistence of the problem without event recording and shows a roughly 12–13 microsecond median handoff-duration difference; it does not establish statistical equivalence, an exact probe-cost bound, or identical channel conditions. The first enabled full radio trace misses application index 1,478, outside its complete internal window. Foreign-beacon and sampled-context pools overflowed and are not used to infer channel occupancy or CCA.

The useful new boundary is after software prepares and enables the MAC: the inspected `lmacTxFrame` normal path configures EDCA and calls `hal_mac_txq_enable` before returning. Most measured timing variation arises downstream of that boundary and is visible in radio cadence, rather than only in completion notification. LMAC return and PPDU programming are not actual RF-start timestamps. The remaining interval includes hardware/MAC scheduling, access to the medium, transmission/acknowledgment, and completion delivery; these traces do not separate CCA deferral, EDCA backoff, hidden MAC queues, or retries. Application core placement and larger application queues are therefore not supported as remedies for this measured dominant delay. A narrower next experiment needs an actual TX-start or medium-busy/backoff observation tied to the same descriptor, rather than another application pacing or generic SDK comparison.

No occupancy gain or production firmware fix is claimed. Final acquisition checks leave the S3 on the pinned original AP, with ready lightweight sensing, CSI enabled, no reported drops, the completion callback unregistered, and internal recording stopped; the C6 is idle. Local artifacts are under `espectre-s3-driver-trace`, including the frozen `v2` image/manifest, `on2`, `off2`, and `on3` acquisitions, driver-to-radio joins, impact comparison, and validation audit. Production firmware sources were not changed by this experiment.

### S3 MAC queue enable and contention controls

A follow-up prioritized an actionable internal wait mechanism before testing other hypotheses. Static inspection identified `hal_mac_tx_config_edca`, `hal_mac_txq_enable`, `hal_mac_txq_disable`, and `hal_mac_set_txq_invalid`. Isolated wrappers recorded the per-packet LMAC contention state, read back the configuration register, and sampled the queue register immediately before and after each wrapped enable/disable/invalid call. Register addresses and bit meanings came from the inspected SDK functions, including its queue-valid and queue-enabled getters. These were read-only observations: the original functions, contention values, and queue operations were forwarded unchanged. No continuously sampled queue state, actual RF-start timestamp, or validated CCA/NAV busy signal was available. The inspected FTM timestamp routine is conditional on FTM action frames and was not treated as a timestamp source for ordinary ICMP traffic.

One completed diagnostic acquisition used the same pinned AP, channel 10, AP-local destination, CSI, lightweight sensing, and 100 pps. The S3 application SHA-256 is `db2e221f7bfa2afbbd0eba4c44993b9d139231f5995bf863a45dc0c404f29693`; effective SDK configuration remains unchanged. The C6 retained the existing 7,000-record observer firmware. The internal first-second window contains 100 uniquely identified ping frames and 1,412 events, with all three exports complete and checksum-valid and no internal buffer overflow. All 100 ping identities were observed on the C6 from their first non-retry transmission, with no observed retransmission inside this window. The full 17-second capture has 1,700 exact APP/HAND/DONE and primary-radio matches. A preceding acquisition attempt stopped before arming because the observer was disconnected; it produced no measurement. The unrelated C5 was excluded from the experiment.

All 100 ping frames used hardware queue 1, an AIFS field of 2, a current/minimum CW exponent of 3, and a maximum CW exponent of 4. The initially programmed random backoff ranged from 0 to 7 slots, and every configuration register readback matched the programmed AIFS and backoff fields. Both queue-valid and queue-enabled bits were set in every post-enable snapshot. Configuration-readback-to-enable took 4–5 microseconds, and the wrapped enable call took 2–3 microseconds. No wrapped queue-disable or queue-invalid call occurred during the internal window. These results do not show a delayed software enable, an observed software disable/re-enable hold, an inflated initial contention count, or a configuration-write mismatch.

The target delay still occurred: 13 of the 100 packets took more than 2 ms from confirmed queue enable to the completion callback; p95 was 3.056 ms, and the maximum was 3.452 ms. Two of these slow packets had an initial backoff count of zero, including the maximum-delay packet. Thus, the loaded initial random count does not by itself account for the observed waiting. A zero initial count does not remove other required channel-access waits, and a configured count is not an elapsed wall-clock backoff measurement. LMAC-return intervals remained 100% regular, while completion and first-radio intervals were both 89.90% regular over the 99 internal pairs. Full first-radio regularity was 88.82%; differences from earlier acquisitions are not attributed to an optimization, since no transmission policy was changed.

The favorable hypothesis did not yield a concrete fix in the observed controls. A deeper hardware/MAC gate remains possible: point-in-time register snapshots and wrapped software calls do not exclude hardware-driven transitions, unobserved control paths, CCA deferral, or NAV-related waiting. No speculative queue or contention change, additional diagnostic repeat, or A/B/A intervention was run. Further attribution requires a validated observation that separates medium-access waiting from internal hardware scheduling. Local evidence is under `espectre-s3-mac-gate`, with the completed acquisition in `diagnostic2`, the frozen application and manifest, exact driver/radio joins, gate analysis, and audit. Final checks leave S3 sensing ready on the pinned AP, CSI active, the completion callback unregistered, internal recording stopped, and the C6 idle. No production firmware sources were changed.

Offline follow-up inspected the SDK's MAC diagnostics and PHY/RF-test libraries without accessing devices or running another capture. `dbg_lmac_diag_statis_dump` exposes register addresses and generic `diag0`–`diag12` labels, but no verified CCA, NAV, or RF-start bit definitions. `librftest.a:BackOffCountGet` initially suggested a candidate count field in the low 12 bits of the EDCA register bank. Deeper inspection rejected that interpretation: `hal_mac_tx_config_timeout` writes those bits from the lifetime handling in `lmacSetTxFrame`. The [public S2 register definitions](https://github.com/esp-rs/esp-pacs/blob/main/esp32s2/src/wifi/tx_slot_config/config.rs) independently label them `TIMEOUT`; the S3 writer, rather than cross-chip analogy, is the confirming evidence. All 100 existing pre-enable snapshots contain 1023 in this field, including all 13 packets later delayed by more than 2 ms. This is not evidence of a residual or elapsed backoff count.

Ghidra 12.1.3 was downloaded from its official release, SHA-256 verified, and its native decompiler built locally. Analysis used the SDK 5.5.5 libraries and the packaged S3 ROM ELF. Direct relocatable-object import reported unsupported Xtensa relocations, so the useful library decompilation used a separate GNU-linked, offline-only ELF with explicit external-symbol placeholders; it is not a firmware image and was never flashed. Read-only literal folding made register accesses explicit, and relevant pseudocode was checked against the disassembly. The ROM provides a second correction: `rom_set_cca` writes the low byte of `0x6001c01c` with a supplied configuration value or `0xbf`, while `phy_get_cca` reads that byte. This getter is not an instantaneous channel-busy flag. The RF-test variable `cca_block_num` is also unsuitable as busy time: `tx_a_frame` increments it after approximately 500 ms without the test completion condition, without establishing why that transmission stalled.

The deeper analysis did identify a separate observable software TX-block control: `pm_off_channel`, `pm_coex_schm_process`, and `pm_coex_slice_timeout_process` set mask `0x000e0000` at `0x60033ca0`, and `hal_pm_unblock_txq` clears it. Its state during the delayed connected transmissions has not been measured. A bounded read-only trace of these controls would test another internal hold mechanism, with sampling gaps and overflow checked before interpreting a negative result. Separately, `phy_set_cca_cnt` and `phy_get_cca_cnt` expose control/result registers at `0x6001d058`, `0x6001d05c`, and `0x6001d060`. Their timing scale, reset/completion behavior, and response to controlled traffic could be characterized experimentally, but no busy/idle interpretation, NAV attribution, or per-packet causality is established. No device access, CCA-policy change, or new capture was performed during this follow-up. Corrected mappings, evidence hashes, and the unexecuted observation protocol are under `espectre-s3-driver-inspection`, in `cca-backoff-feasibility.json`, `decompiled/`, and `next-observation-plan.json`.

A diagnostic firmware for the TX-block observation was subsequently prepared and built, but not flashed. The isolated application is under `espectre-s3-tx-block/firmware/`, SHA-256 `4ea8431bfa5b65b85823e139183b3637c4275fff3f624bc9ed7fcf3b45068544`, with a frozen manifest and unchanged effective SDK configuration. Its only source change relative to the preceding MAC-gate probe is `outbound_probe.cpp`. It reads `0x60033ca0` before and after queue enable and samples the same register on core 1 at priority 1, targeting 50-microsecond intervals for 1.02 seconds. Masked-state changes, read brackets, and gaps exceeding 100 microseconds share the existing indexed, checksummed trace buffer. The sampler is stopped and joined before export; it performs no MMIO writes. Binary inspection verified the sampler's IRAM placement, inline atomic operations, and register load. The additional task requests a 2,048-byte stack; actual heap margin, stack margin, sample coverage, and timing impact remain unmeasured. This temporary probe is guarded for dual-core S3 and does not alter the portable production implementation.

The accompanying analyzer rejects overflow, conflicting metadata, missing records, and missing queue-boundary snapshots. It labels incompletely covered packet waits as inconclusive, and does not treat an unobserved assertion as exclusion of pulses shorter than 100 microseconds, other hardware gates, CCA, or NAV. Fifteen offline checks passed, including replay of the existing 100 packet identities with synthetic block states, simulated gaps and corruption, timer wrap in block classification, disabled recording, and execution through the existing gate parser and new block analyzer. These checks validate analysis behavior, not new RF evidence or an occupancy improvement. No device was accessed during preparation.

The prepared TX-block firmware was then flashed and verified for an authorized capture on the same pinned BSSID, channel 10, and AP-local destination. Its first-second trace contains 1,833 records without overflow, with three complete checksum-valid exports. Of 100 internally identified ping frames, the first lacks a captured HAND entry at the leading window boundary and is explicitly excluded; all remaining 99 have exact software and C6 radio matches. The sampler completed 17,419 reads, recorded one initial masked state and no masked-state changes, and retained 1,324 bytes of stack margin. It also recorded 211 gaps above 100 microseconds, with a maximum of 4,271 microseconds. Therefore, 13 packet waits are inconclusive. The other 86 waits include 11 delays above 2 ms, ranging from 2,060 to 5,141 microseconds, with no assertion of the inspected block bits throughout their verified coverage. This control does not explain those covered delays at 100-microsecond resolution; shorter pulses and other hardware gates remain outside the inference.

The full 17-second observation retained 1,699 exact application/handoff/completion/radio matches. A subsequent control with internal recording disabled retained 1,700 software matches and 1,699 radio matches, with one explicitly missing radio identity. Handoff intervals were 99.82% and 100% within 8–12 ms, while first-radio intervals were 84.57% and 88.10%, respectively. These sequential runs demonstrate that outgoing irregularity persists with the sampler disabled; their difference is not an isolated estimate of sampler overhead or an occupancy improvement. The internal on-window contains one observed retransmission, so completion time is not uniformly equivalent to first-transmission time. Local acquisitions, strict block classification, boundary exclusions, and a nine-check evidence audit are under `espectre-s3-tx-block/on1`, `off1`, and `block-result.json`. The next observation targets calibration of the separate PHY counters, without changing CCA thresholds or access policy.

The PHY counter calibration subsequently established a usable timing scale. Two repetitions each of requested counts 40,000, 160,000, and 640,000 completed within observed brackets containing 0.5, 2, and 8 ms, respectively. The first result counter advanced at approximately 80 ticks per microsecond and stopped at the requested value; every trial observed a fresh running state followed by completion. These observations establish repeatable restart and completion behavior for the tested counts, without changing CCA thresholds or access policy. The second counter was then checked against a separate radio source: the C6 sent recognizable, non-QoS broadcast test frames using its own transmitter MAC, with zero Duration fields and the pinned BSSID. The S3 independently received 80 frames in each 800-ms loaded window and verified legacy 1 Mbit/s delivery. Switching from 96-byte to 512-byte transmitted frames increased observed airtime by 266.24 ms and increased the second counter by an average 266.47 ms. Three idle controls measured 181.07–193.16 ms in the second counter; subtracting the injected airtime from loaded trials left 179.63–202.74 ms. This validates its response to occupied radio airtime in the tested conditions, rather than an idle-time or software-queue interpretation. The close mean difference is not a claim of 0.09% absolute measurement accuracy under varying background traffic. Two preceding C6 attempts stopped at unsupported pre-start rate-configuration calls, before transmitting test traffic; configuration after Wi-Fi start succeeded, and actual rate was verified independently. Evidence is under `espectre-s3-cca-counter/calibration1` and `espectre-s3-cca-wait/radio-calibration3`.

A separate passive capture then joined counter snapshots at queue enable and completion to 100 exact S3/C6 ping identities. All counter endpoints were complete and inside the running 1.2-second counter window. Fifteen packets waited more than 2 ms after enable, with a median of 2.949 ms and a maximum of 3.845 ms. The calibrated second counter accounted for a median 90.7% of their measured counter interval. Independent C6 records identified a preceding 1 Mbit/s beacon for each slow packet: 14 first transmissions occurred within 93–255 microseconds of the preceding beacon timestamp plus its MAC serialization duration. Mixed-PHY timestamp offsets prevent interpreting that expression as an exact PPDU-end timestamp, but the observed release timing follows both beacon lengths, 408 and 436 bytes. Six BSSIDs contributed 56 beacons in the covered internal radio interval, an estimated 20.18% of airtime including long preambles. Other-beacon storage filled only after this internal window, so the analysis does not extrapolate its partial full-capture pool. Two slow packets had independently observed retransmissions. One additional packet had at least 0.840 ms between its first radio observation and the completion callback under the independently constrained clock mapping. Excluding these three exceptions, the remaining 12 slow packets retained only 0.280 ms median and 0.477 ms maximum after subtracting measured occupied airtime.

These measurements identify medium-access deferral during real radio activity, particularly slow periodic beacons, as the dominant cause of the observed outgoing delays. They do not support a millisecond software queue hold or the inspected PM/coexistence block as the dominant explanation. They also do not claim to decode every CCA/NAV condition or attribute every outlier to one mechanism. The relevant sources are `espectre-s3-cca-wait/on1`, `radio-calibration-result.json`, and `beacon-correlation.json`; the S3 image SHA-256 is `58fca41e4a25980b444acb9c208eba11cb48f4bd00ea6ca3944a94da284b656a`. No occupancy improvement or production fix follows from this diagnosis alone. A beacon-relative pacing mitigation remains an experiment until tested against unchanged-grid controls.

A first mitigation used a 10.24-ms send period, approximately 97.66 pps, with a phase selected from 40 locally observed beacons across the six BSSIDs. The 100-Hz sensing grid remained unchanged. In the bounded A/B/A comparison, first-radio intervals within 8–12 ms changed from 77.44% to 92.28% and back to 77.73%; their p95 changed from 15.14 to 11.51 and back to 15.05 ms. Occupancy did not follow that improvement: full-window telemetry averaged 93.94%, 91.13%, and 85.00%, respectively. The final default-period command did not restore the original absolute send phase, so this comparison demonstrates reversible radio regularity improvement, but does not establish an occupancy benefit. The beacon phase model also depends on timestamp and callback-latency assumptions; its simulated regularity was not an achieved measurement. These results are under `espectre-s3-beacon-pace`, with image SHA-256 `47db173c913b3454934d75bb639eab1c3a8077d45372e2cac0e389e68d4dd134`.

A second A/B/A experiment compared fixed 100, 125, and 100 pps, explicitly restoring the same absolute 10-ms send phase in both controls and retaining the same association and generator instance. The three 12-second windows measured exactly 100, 125, and 100 application sends per second. Eleven complete one-second telemetry samples per condition gave mean occupancy of 84.82%, 91.45%, and 90.27%, with minima of 83%, 89%, and 86%. Uplink gap p95 was 14.00, 11.52, and 14.83 ms; accepted-RX gap p95 was 16.10, 16.33, and 18.24 ms. The 125-pps condition reduced the fraction of accepted-RX intervals exceeding 10 ms from 46.33%/45.00% in the controls to 13.21%, but the differing control occupancies prevent attributing the entire occupancy difference to the extra traffic. All software identities matched; one radio identity was missing in the first control, while the other two had complete radio matches. The cumulative drop counter was constant within each scored telemetry interval, at zero in the first control and two in the later conditions; this does not imply zero drops between trials. Evidence is under `espectre-s3-fixed-margin`, with image SHA-256 `5ed990cae5bf6901a5e58ca9820b10551917a69886269f9e7afb286799ce208e`. This is fixed operator-selected oversupply, with 25% more request/reply traffic, rather than an occupancy-driven rate controller or a production default change.

A third experiment evaluated the operator's proposed 200-pps oversupply and variable pacing while keeping the sensing grid at 100 Hz. One isolated S3 image selected either a fixed 5-ms period or a deterministic shuffled sequence containing eight 9-ms, sixteen 10-ms, and eight 11-ms intervals per 320-ms cycle. This balanced sequence targets 100 pps without cumulative drift; actual wake-up times retain the existing millisecond scheduler quantization. Both modes skip sufficiently late deadlines rather than sending catch-up bursts. The sequence was `100 / 200 / variable / 200 / variable / 100`, with the same association, generator instance, and absolute fixed-control send phase. Each S3 capture lasted eight seconds plus two seconds of completion/RX grace; the C6 capture was shortened to twelve seconds to keep the existing target-packet buffers below capacity at 200 pps. Each condition retained seven complete one-second occupancy observations. CSI remained enabled, and no HTTP load was added.

| Pacing | Actual application pps, first / second trial | Mean occupancy, first / second trial | Minimum occupancy, first / second trial | Accepted-RX gap p95, first / second trial |
| --- | --- | --- | --- | --- |
| Fixed 100 pps controls | 100.00 / 100.00 | 88.00% / 91.57% | 83% / 87% | 18.60 / 17.80 ms |
| Fixed 200 pps | 199.88 / 200.00 | 90.43% / 92.29% | 88% / 90% | 9.38 / 9.43 ms |
| Balanced 9/10/11-ms intervals | 100.00 / 100.00 | 87.14% / 84.86% | 85% / 82% | 17.69 / 17.98 ms |

The 200-pps trials clearly shortened accepted-RX gap tails, but did not establish a robust occupancy improvement: their pooled mean was 91.36%, compared with 89.79% for the controls, while the two control means differed by 3.57 points. This result does not justify doubling request/reply traffic as the production default. The tested variable sequence had lower occupancy than both fixed controls, so it is not a demonstrated mitigation; other randomization amplitudes and distributions were not tested. These are short S3/AP-local trials, not long-duration performance gates or a paired comparison with C5. All six captures had complete software identity matching and zero S3 trace or C6 target-packet overflow. The second variable trial lacked one of 800 radio identities; all other radio identities matched. One malformed serial export line in the final control was recovered from checksum-valid redundant copies. Auxiliary other-beacon pools remained bounded and are not used to claim complete background traffic coverage. Cumulative drop counters stayed constant within each scored telemetry interval, but rose between some conditions. A 62-check audit verified frozen images and sources, raw hashes, stream checksums, rate and grid invariants, and final device state. Evidence is under `espectre-s3-rate-dither`, including `result.json`, `comparison.json`, and `audit.json`; S3 image SHA-256 is `41bd51f9ef787307211bd86a331a77f110ffb32dc502ce9f7423dbcdcbb88bb2`, and C6 image SHA-256 is `ff6b11e39f8e367fa7fa0629d4a7ce345367eb576e5691310789e762fbe9df4f`. The S3 was left at fixed 100 pps with CSI active and capture stopped; the C6 was left idle. No production source changes were made for these experiments.

The operator separately reported C5 receive occupancy of 93% with ping, 84% with DNS UDP, and 86% with DNS TCP on the same BSSID. These observations motivate a chip/driver/AP-interaction hypothesis, but are not a controlled paired comparison, and receive occupancy is not directly interchangeable with S3 outgoing interval regularity. The C5 remained excluded from all operations.

### Timing and admission findings

The packet's useful sensing instant is reception by the device, when the PHY estimates CSI from HT-LTF. AP transmit time is not directly available to the device, and callback processing time describes software latency rather than the RF channel sample. A CSI result cannot be deferred arbitrarily and recalculated later without retaining the original RF samples; the callback exposes the estimate already associated with that received packet.

ESP-IDF processing therefore uses `rx_ctrl.timestamp` for temporal slots, gaps, and occupancy. The Wi-Fi timestamp belongs to the MAC clock domain, not the `esp_timer` domain. The runtime records `esp_timer` at callback acceptance, computes callback-to-loop queue age only within that software clock, and translates the elapsed duration into the RX timestamp domain for backlog rejection. Directly comparing the two absolute clocks is invalid and was the reason target guards existed in the earlier implementation.

Packets counted as same-slot excess still contain valid CSI, but they do not add a new temporal position to the fixed sensing grid. Sensing frontends admit the candidate nearest the slot center rather than letting a burst fill a physical-time window. Raw HTTP preserves classified timestamped CSI before temporal admission so research capture does not discard those frames; live detectors and derived sensing views apply the same admission as deployed sensing frontends.

## Decision

Standardize managed CSI traffic as follows:

- expose three explicit internal generator values: `ping` for stateless ICMP echo, `dns` for connectionless UDP/53 queries, and `dns_tcp` for length-prefixed queries over one persistent, non-blocking TCP connection with `TCP_NODELAY` and reconnect backoff;
- keep `ping` as the shared schema default and the published Native, Matter, ESPHome, and Micro-ESPectre product configuration default; keep `dns` and `dns_tcp` as explicit operator selections;
- preserve a configured or persisted selection without automatic protocol fallback, so operators can choose the source that works in their device, driver, AP, and resolver context;
- request DSCP 46 treatment for internal traffic and the standalone external UDP tool, without treating a particular WMM TID or occupancy improvement as guaranteed;
- preserve the configured send phase through ordinary scheduler jitter, but restart from the actual send time when the next phase deadline would be less than half a period away, so no generator emits a close catch-up pair;
- apply that fixed-phase rule in the shared C++ generator, the Micro-ESPectre native generator, and `tools/espectre_traffic_generator.py`;
- limit pacing multicast to the local link and prefer unicast or the joined multicast group over subnet or limited broadcast;
- in ESP-IDF `external` mode, admit unicast ICMP Echo Requests addressed to the device as well as canonical UDP markers, while keeping the internal generator stopped;
- keep occupancy diagnostic-only and never make device send rate chase admitted occupancy;
- place CSI on the detector grid using the device Wi-Fi RX timestamp, with processing time used only for same-domain queue-age measurement; and
- keep raw HTTP records even when the sensing view classifies additional same-slot records as excess.

Both DNS modes require a gateway resolver on port `53`; `dns_tcp` additionally requires TCP query support. If the selected mode is unsuitable, operators must select another explicit mode rather than relying on silent fallback. Micro-ESPectre exposes all three modes as deployment settings and currently selects `ping` in its committed `config.py`.

## Decision History

| Date | Direction | Resolution |
| --- | --- | --- |
| 2026-08-23 | Use ICMP ping or UDP/53 DNS as interchangeable internal generators | Ping retained; UDP/53 rejected after the C3 capture showed legacy PHY and extreme observed retry share |
| 2026-08-23 | Mark managed traffic for low latency | Retained as a request; external DSCP 46 changed WMM category but did not materially change retry or timing on the tested AP |
| 2026-08-23 | Skip only deadlines already fully missed on host pacing | Replaced by the half-period reset rule so late wake-up cannot produce a close packet pair |
| 2026-08-23 | Compare Wi-Fi RX time directly with the processing wall clock on selected targets | Replaced by measuring queue age in `esp_timer` and translating only that duration into the MAC timestamp domain |
| 2026-08-23 | Use persistent non-blocking DNS/TCP and one pacing policy across firmware and host tools | Retained as the optional `dns_tcp` path |
| 2026-08-29 | Admit external ICMP as a separate traffic-mode value | Rejected; `external` already denotes external ownership and now accepts either canonical UDP markers or unicast Echo Requests |
| 2026-08-29 | Ban DNS/UDP globally after the C3 result | Replaced by explicit `ping`, `dns`, and `dns_tcp` choices after classic ESP32 DNS/UDP remained stable in Direct and monitor-mode tests |
| 2026-08-30 | Keep Micro-ESPectre ping-only | Replaced by deployment-time `ping`, `dns`, and `dns_tcp` selection through the shared native generator; the committed Micro configuration later selected `dns` |
| 2026-08-31 | Keep classic ESP32, ESP32-S2, and Micro-ESPectre on `dns` | Replaced; published product configurations now use the shared `ping` default |
| 2026-09-06 | Move internal S3 ping from Video to Voice | Keep DSCP 46; the bounded A/B/A trial showed only about 0.8 points of RX regularity improvement, and occupancy remained elevated after returning to Video |
| 2026-09-06 | Switch S3 to DNS/TCP for higher occupancy | Keep explicit protocol selection; occupancy remained elevated on returning to ping, while DNS/TCP had lower CSI supply and mixed interval tails |
| 2026-09-06 | Replace S3 dynamic32 TX buffers with static32 | Retain dynamic32; the qualified comparison found only a small uplink regularity difference, no occupancy gain, and approximately 54 KiB less free heap |
| 2026-09-06 | Bypass S3 sensing to regularize the uplink | Retain the sensing path; callback time fell from about 224 to 42 µs without a demonstrated uplink regularity gain, while most observed irregularity arose after the measured driver handoff |
| 2026-09-06 | Disable S3 CSI to isolate outbound timing | Retain CSI; the qualified baseline improvement persisted after re-enabling CSI, while C6 observation gaps prevented an under-load A/B/A conclusion |
| 2026-09-06 | Add S3 TX-completion timing to isolate uplink irregularity | Keep production settings; exact packet joins locate the main cadence change after driver handoff and before or at radio transmission, without separating driver/MAC scheduling from shared-channel access |

## Alternatives Considered

### Make DNS/UDP the universal default

Rejected. UDP/53 triggered a target-specific Wi-Fi handling path on the tested C3 that changed QoS, forced predominantly legacy transmission, added RTS/CTS and retries, and disrupted the response cadence. It was stable on the tested classic ESP32, but that result does not make it portable. Explicit selection and target-specific defaults retain both outcomes without pretending one mode is universal.

### Make DNS/TCP the universal default

Rejected. Ping is stateless, broadly available, and does not require a TCP-capable gateway resolver. DNS/TCP remains valuable as an alternative traffic shape and performed well on the tested gateway, but it has connection lifecycle and compatibility costs.

### Use only external UDP

Rejected. External pacing performed well and is useful for controlled experiments, but it requires another always-on host and correct routing. Micro-ESPectre intentionally has no external UDP listener. Its device-local generator supports `ping`, `dns`, and `dns_tcp` without requiring an ESPectre-specific host service.

### Adapt send rate from occupancy

Rejected. Occupancy is capped by the sampler and mixes traffic supply with AP scheduling and slot placement. Earlier bounded C3 and classic ESP32 trials did not outperform fixed cadence. Raw HTTP reports transport backpressure but never changes the external generator rate.

### Admit every received CSI packet

Rejected for live sensing. A burst would manufacture a full detector window without covering the corresponding physical time. Valid excess CSI remains available through raw HTTP capture when research needs it.

### Timestamp by AP transmit time or processing time

Rejected. AP transmit time is not available as the device's authoritative sensing timestamp, while processing time measures software scheduling after the PHY observation. The Wi-Fi RX timestamp best represents when the measured channel existed.

## Consequences

Benefits:

- users can select among stateless ICMP, connectionless DNS/UDP, and stateful DNS/TCP without installing an external network service;
- target product defaults can follow measured device behavior without removing alternatives from other targets;
- generator-side scheduler delay no longer creates avoidable catch-up bursts;
- ping, DNS/UDP, DNS/TCP, external UDP, and external ICMP have explicit and comparable roles;
- all production paths use the same physical-time interpretation for motion sensing;
- DNS modes preserve the AP as the downlink packet source without assuming UDP or TCP is universally healthier; and
- raw HTTP capture remains lossless with respect to temporal admission decisions, except for explicitly counted bounded-ring drops.

Trade-offs and limits:

- DNS/TCP maintains socket state and reconnect logic and depends on gateway TCP/53 support;
- DNS/UDP may enter a target-specific legacy-PHY and retry path, as measured on the C3;
- closely spaced eligible responses may create same-slot excess even when occupancy improves;
- DSCP-to-WMM mapping is direction-, sender-, AP-, and driver-dependent;
- external ICMP requires no ESPectre-specific host service, but its Echo Replies add device-originated traffic and host pacing remains outside the firmware;
- the AP may still queue, aggregate, retry, or burst frames after the generator has paced them;
- monitor-mode Retry shares are observer measurements, not authoritative device or AP counters;
- pcap gap statistics use the monitor's observation timestamps, not the device's authoritative `rx_ctrl.timestamp`;
- encrypted captures cannot prove each frame's L4 protocol without correlating the controlled test mode; and
- the numeric capture results characterize one C3 and one classic ESP32 on one AP in separate laboratory intervals, so future devices and APs require the same validation rather than copied thresholds.

## Related

- [`2026-08-15-use-fixed-temporal-csi-admission.md`](2026-08-15-use-fixed-temporal-csi-admission.md)
- [`../TUNING.md`](../TUNING.md)
- [`../SETUP.md`](../SETUP.md)
- [`../ALGORITHMS.md`](../ALGORITHMS.md)
- [`../CLI.md`](../CLI.md)
- [`../API.md`](../API.md)
- [`2026-07-03-unify-raw-csi-collection-over-http.md`](2026-07-03-unify-raw-csi-collection-over-http.md)
