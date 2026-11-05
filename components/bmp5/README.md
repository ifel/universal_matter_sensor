# bmp5

Bosch Sensortec [BMP5 SensorAPI](https://github.com/boschsensortec/BMP5_SensorAPI) (BMP581 / BMP585,
BSD-3-Clause) as a git submodule in `third_party/`, plus `bmp5_i2c.c` glue mapping its read/write/delay
callbacks onto esp-idf-lib `i2cdev`.

After cloning: `git submodule update --init` (or clone with `--recursive`).
To update upstream: `git -C components/bmp5/third_party fetch && git -C components/bmp5/third_party checkout <commit>`,
then commit the new submodule pointer.

Known upstream issue (clang-analyzer): `bmp5_get_power_mode()` can return `BMP5_OK` without setting
`*powermode` when the chip is in standby with deep standby enabled but deep-standby conditions not met
(e.g. IIR enabled); callers then read an uninitialized value. Not reached with this project's config
(power-on state meets the conditions; Bosch's standby transition disables deep standby).
