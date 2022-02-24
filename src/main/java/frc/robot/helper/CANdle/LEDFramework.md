# LED Framework

An explanation of the LED API.

[//]: # (🟠🟡🟢🟣⚫️⚪️🟤)
## LED Raid Controller

This writes multiple `LED Ranges` to the physical addressable LED strip. 
The `LED Ranges` can be different lengths.  
`[ 🟠🟠🟠🟣🟣🟣🟢🟢🟢 | 🟠🟣🟢 | 🟠🟠🟠🟣🟣🟣🟢🟢🟢 | 🟠🟣🟢 ]`

## LED Range

A length of n LEDs that defines where on the robot it is located.

`[🟠🟠🟠🟣🟣🟣🟢🟢🟢]`

## LED Section

An LED Section includes percentage that defines the relative size and location on each range and contains 1 `Pattern Generator`,

`🟠🟠🟠`

## LED Instruction

A data structure that defines a RGBW color and a block of LEDs to set to that color.
```
R: 0
G: 255
B: 0
W: 0

startIndex: 0
count: 5
```
Sets all leds from indices 0-4 to 100% Green 

## Pattern Generators

`Pattern Generators` programmatically generates LED instructions that control the LEDs based on its state.
```
[
    1st Ball Color: RED,
]  
```
Turns into  
`[0%-100%🟠]`

## Subsystem

Some subsystems will give their state to pattern generators.  
```
[
    1st Ball Color: RED,
    2nd Ball Color: Blue,
    Auto-aim:       on,
]
```
