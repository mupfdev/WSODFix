# WSODFix

<p>
<a href="https://github.com/mupfelofen-de/WSODFix">
    <img src="https://img.shields.io/badge/project-GitHub-blue?style=flat?svg=true" alt="GitHub project" />
</a>
<a href="https://github.com/mupfelofen-de/eszFW/blob/master/LICENSE.md">
    <img src="https://img.shields.io/badge/licence-MIT-blue?style=flat?svg=true" alt="Licence" />
</a>
<a href="https://www.travis-ci.com/mupfelofen-de/WSODFix">
    <img src="https://www.travis-ci.com/mupfelofen-de/WSODFix.svg?branch=master" alt="Build status" />
</a>
</p>

## About

Nokia mobile phones such as the N-Gage running early versions of the
Symbian OS suffer from a very common problem widely known as the White
Screen of Death or simply WSOD.  This happens when the internal memory
of the phone becomes so full that the unit can no longer boot and
becomes virtually unusable.

To solve this, the memory, referred to here as the user area, must be
formatted.

However, professional flashing equipment is needed to achieve this and
nowadays it is hard to find such equipment for sale.  More than often,
manufacturers, which cloned official service hardware, sold it in
addition to hardware licences in the form of a dongle.  Also, as new
devices would come out, old ones would literally be rendered unusable by
the manufacturers via the internet or sometimes by malware-infected
software released by competitors.

There is official software with which some of these devices can be
repaired and reactivated with, but access to reliable information is
anything but trivial.

This project is not an alternative to professional flashing equipment,
but aims at offering the possibility to make older Symbian phones
affected by a simple WSOD usable again by leveraging open-source
software and DIY hardware.

The project is based on a STM32F103C8 microcontroller board a.k.a. a
[blue pill board](http://reblag.dk/stm32/).

## Status

The project is currently in a prototype phase. The current development
state is neither complete nor functional.

## Protocol specification

Tbd.

## Installation

1. Install [PlatformIO Core](http://docs.platformio.org/page/core.html)
2. Run these commands:

```bash
    # Build project
    > platformio run

    # Upload firmware
    > platformio run --target upload

    # Clean build files (optional)
    > platformio run --target clean
```

## Licence

This project is licensed under the "The MIT License".  See the file
[LICENSE.md](LICENSE.md) for details.
