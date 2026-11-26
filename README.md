# HydraSDR

<p align="center">
  <img src="assets/hydrasdr-logo.png" alt="HydraSDR logo" width="360">
</p>

HydraSDR turns supported MediaTek Wi-Fi hardware into a multi-RX / multi-TX SDR platform.
The current backend targets the MT7921AUN / MT7961 architecture family with MT7972 RF.

> **Note:** HydraSDR is experimental software.

## Features

- Native complex I/Q RX at 10, 20, 40 and 80 MS/s
- Coherent **2RX at 2 × 40 MS/s**
- Experimental arbitrary-I/Q TX
- Coherent same-channel **2TX**
- Simultaneous **2TX + 2RX**
- ~**4.1 MS/s** validated RF-effective TX update rate on the current PHY scalar path
- Shared-LO phase-coherent operation
- Live tuning and gain control
- Low-level PHY/RF diagnostics and register access
- SoapySDR support via `driver=hydra`
- Custom open N9 firmware and watchdog-protected TX experiments

## Ongoing Research

- Higher-rate / wideband TX through native PHY FIFO/AWG paths
- Independent I/Q streams for multiple TX chains
- Digital beamforming with per-chain phase/amplitude control
- DBDC / simultaneous multi-frequency TX
- TX ICAP and internal observation-path mapping
- Full-duplex TX/RX characterization
- External RF power, spectrum and EVM validation
- Support for additional MediaTek families, including MT7925 and MT799x
- Longer-term support for suitable RF-capable devices from other vendors

## Supported Hardware

| Hardware | USB ID | Status |
| --- | --- | --- |
| **Fenvi AX1800 / MT7921AUN + MT7972** | `0e8d:7961` | **Primary development target** |
| COMFAST MT7961/MT7972 USB adapter | `3574:6211` | Supported |
| EDUP AX3000/AX3000M MT7961/MT7972 | `0e8d:7961` | Supported with board-specific limits |
| Generic MT7961/MT7972 board | `0e8d:7961` | Provisional |
| MT7925 USB | `0e8d:7925` | Discovery only |
| Netgear A8500 / MT7925 | `0846:9050` | Discovery only |
| Netgear A9000 / MT7925 | `0846:9072` | Discovery only |

The Fenvi AX1800 is sold as an MT7921AUN device and uses the same MT7961-family digital/firmware architecture targeted by the current backend. Board-level RF routing and calibration can still differ between products.

## Build

Requirements: CMake, a C11 compiler, pthreads and libusb-1.0 development headers. SoapySDR is optional.

```sh
git clone https://github.com/mbound/hydraSDR.git
cd hydraSDR
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Disable SoapySDR with:

```sh
cmake -S . -B build -DHYDRASDR_SOAPY=OFF
```

## Usage

List devices:

```sh
./build/hydra_rx --list
```

Single RX at 40 MS/s:

```sh
./build/hydra_rx --frequency 2437000000 --rate 40000000 rx.cs16 4000000
```

Dual RX at 2 × 40 MS/s:

```sh
./build/hydra_dual_rx 2437000000 40000000 204800 rx0.cs16 rx1.cs16
```

SoapySDR discovery:

```sh
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy SoapySDRUtil --find=driver=hydra
```

The public C API is in `include/hydrasdr.h`. Experimental TX controls are in `include/hydra_tx_control.h`.


## Credits

HydraSDR is derived from [BirriaSDR](https://github.com/luigifcruz/BirriaSDR), whose MT7921 SDR work provided the original user-space USB, RX and firmware foundation.

Additional work draws on OpenWrt `mt76`, `w1700k-sdr`, Mtk79ICAP and librtlsdr. See `NOTICE` and `LICENSES/` for attribution and license details.

## License

MIT. See `LICENSE`, `NOTICE` and `LICENSES/`.
