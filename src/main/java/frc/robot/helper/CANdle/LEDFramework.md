# LED Framework

An explanation of the LED API.

[//]: # (🟠🟡🟢🟣⚫️⚪️🟤)
## LED Raid Controller

This writes multiple `LED Ranges` to the physical addressable LED strip. 
The `LED Ranges` can be different lengths.  
`[ 🟠🟠🟠🟣🟣🟣🟢🟢🟢 | 🟠🟣🟢 | 🟠🟠🟠🟣🟣🟣🟢🟢🟢 | 🟠🟣🟢 ]`

## LED Section

A length of n LEDs that each contain `Pattern Generators`,

`[🟠🟠🟠🟣🟣🟣🟢🟢🟢]`


## Pattern Generators

`Pattern Generators` programmatically generates LED colors based on a subsystem's state.
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
