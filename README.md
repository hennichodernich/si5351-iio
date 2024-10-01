# si5351-iio

This is an Industrial IO (IIO) driver for the Si5351 chips in MSOP and QFN packages. It uses code snippets from an older version of the clk-si5351.c kernel driver.

The corresponding devicetree entry may look like this:
```
clocksynth: si5351-iio@60 {
        compatible = "silabs,si5351a";
        reg = <0x60>;
    };
```

Optional parameters are

- "devname" (string)
- "xtal-freq" (int)
- "quadrature-mode" (bool)

"devname" creates an arbitrary name for the device name e.g. in iio_info.
"xtal-freq" has to be used if an input clock is used that isn't 25MHz.
"quadrature-mode" locks outputs 0 and 1 to the same frequency and exactly 90 degrees phase shift. Output 2 is unused in this case.
