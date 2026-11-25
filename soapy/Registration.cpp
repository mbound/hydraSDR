/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The HydraSDR Authors
 */
#include "SoapyHydra.hpp"
#include <SoapySDR/Registry.hpp>
#include <cstdio>

/* Enumeration reads USB descriptors only. Nothing is opened or claimed, so
 * qualified-but-busy boards still show up and a bad board cannot be written to.
 */
static SoapySDR::KwargsList findHydra(const SoapySDR::Kwargs &args)
{
    SoapySDR::KwargsList results;
    const int count = hydrasdr_get_device_count();
    for (int i = 0; i < count; ++i) {
        struct hydrasdr_device_info info;
        if (hydrasdr_get_device_info(i, &info) != 0 || !(info.capabilities & HYDRASDR_CAP_RX))
            continue;
        SoapySDR::Kwargs entry;
        entry["driver"] = "hydra";
        entry["index"] = std::to_string(i);
        entry["label"] = std::string(info.name) + " :: bus " + std::to_string(info.bus) +
                         " addr " + std::to_string(info.address);
        char usbid[16];
        snprintf(usbid, sizeof(usbid), "%04x:%04x", info.vid, info.pid);
        entry["usb_id"] = usbid;
        const auto want = args.find("index");
        if (want != args.end() && want->second != entry["index"])
            continue;
        results.push_back(entry);
    }
    return results;
}

/* Open the dongle named by `index`, or the first one when it is absent. */
static SoapySDR::Device *makeHydra(const SoapySDR::Kwargs &args)
{
    const auto it = args.find("index");
    return new SoapyHydra(it == args.end() ? 0 : std::stoul(it->second), args);
}

static SoapySDR::Registry registerHydra("hydra", &findHydra, &makeHydra, SOAPY_SDR_ABI_VERSION);
