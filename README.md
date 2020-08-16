# PMS700x

Embeded-hal implementation for the PMS700x family of sensors

## Example

```rust
let serial = get_embed_hal_serial_from_somewhere();

let mut pms = Pms700X::new(serial).into_active()?;
let result = nb::block!(pms.read())?;

let concentration_10 = result.pm10;
let concentration_25 = result.pm25;
let concentration_100 = result.pm100;
```