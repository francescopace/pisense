/*
 * Browser-only dependency boundary for the ESPectre Web Serial wizard.
 * The Rollup output contains protocol and flashing code, but no third-party UI.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

import { ImprovSerial as BaseImprovSerial } from 'improv-wifi-serial-sdk/dist/serial.js';
import { PortNotReady } from 'improv-wifi-serial-sdk/dist/const.js';
import { Transport as BaseTransport } from 'esptool-js';

export { ESPLoader } from 'esptool-js';
export { ImprovSerialCurrentState } from 'improv-wifi-serial-sdk/dist/const.js';

const ESPECTRE_IMPROV_GET_MATTER_ONBOARDING = 0x80;

export class Transport extends BaseTransport {
    trace(message) {
        if (this.tracing) super.trace(message);
    }

    async readLoop() {
        while (this.device.readable) {
            const reader = this.device.readable.getReader();
            this.reader = reader;
            try {
                while (true) {
                    const { value, done } = await reader.read();
                    if (done) return;
                    if (value?.length) {
                        this.buffer = this.appendArray(this.buffer, Uint8Array.from(value));
                    }
                }
            } catch (error) {
                const recoverable = ['BufferOverrunError', 'FramingError', 'BreakError', 'ParityError']
                    .includes(error?.name);
                if (!recoverable) {
                    this.onDeviceLostCallback?.();
                    return;
                }
            } finally {
                reader.releaseLock();
                if (this.reader === reader) this.reader = undefined;
            }
        }
    }
}

export class ImprovSerial extends BaseImprovSerial {
    async initialize(timeout = 1000) {
        // SDK 2.8.1 uses an async Promise executor here, which loses RPC errors.
        const input = this._processInput();
        await Promise.resolve();
        if (this._reader === undefined) {
            await input;
            throw new PortNotReady();
        }

        let timer;
        let retryInterval;
        const state = this.requestCurrentState();
        try {
            await Promise.race([
                state,
                new Promise((resolve, reject) => {
                    timer = setTimeout(() => reject(new Error('Improv Wi-Fi Serial not detected')), timeout);
                    retryInterval = setInterval(() => this._sendRPC(2, []), 1000);
                }),
            ]);
            clearInterval(retryInterval);
            clearTimeout(timer);
            await this.requestInfo();
            return this.info;
        } catch (error) {
            clearInterval(retryInterval);
            clearTimeout(timer);
            // Settle the SDK command before closing, releasing its timer and listener.
            this._rpcFeedback?.reject(error);
            await state.catch(() => {});
            await this._rpcLock;
            await this.close();
            throw error;
        } finally {
            clearInterval(retryInterval);
            clearTimeout(timer);
        }
    }

    async requestMatterOnboarding(timeout) {
        const response = await this._sendRPCWithResponse(
            ESPECTRE_IMPROV_GET_MATTER_ONBOARDING, [], timeout
        );
        return {
            qr: response[0] || '',
            manual: response[1] || '',
        };
    }
}
