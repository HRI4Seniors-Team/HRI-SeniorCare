# 2026-10-12 Current Device Exact Firmware

- Status: latest verified firmware
- Board: `bread-compact-wifi`
- Project: `xiaozhi`
- Application version: `2.0.3`
- Compile time: `Oct 6 2026 19:59:58`
- ESP-IDF: `v5.5.4`
- ELF SHA-256: `9679efb0f21ee0e9ddd89594de1504aaf070f24adeb1486169b888b711d5867c`
- Application image: `flash/xiaozhi.bin`
- Application size: `4,173,168` bytes
- Application SHA-256: `C80CC63115364A3E1A906BC2A042E3EFBE387652D68E1F0F36EA0400A4567234`
- Verification: byte-for-byte match with the application image read from COM11
- Source package: `HRI-SeniorCare-main.zip`

Flash offsets:

- `0x0000 flash/bootloader/bootloader.bin`
- `0x8000 flash/partition_table/partition-table.bin`
- `0xD000 flash/ota_data_initial.bin`
- `0x20000 flash/xiaozhi.bin`
- `0x820000 flash/generated_assets.bin`

Use `flash/flash_args` or `flash/flasher_args.json` for the complete flashing layout.
