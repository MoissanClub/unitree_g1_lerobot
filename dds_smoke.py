#!/usr/bin/env python3
"""Unitree DDS loopback smoke test with the tracing block removed."""

import unitree_sdk2py.core.channel as channel

channel.ChannelConfigHasInterface = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>"""

channel.ChannelFactoryInitialize(0, "lo")
print("DDS OK")
