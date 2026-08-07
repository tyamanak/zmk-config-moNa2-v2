# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_MAGNITUDE_SCALER app PRIVATE src/input_processor_magnitude_scaler.c)
