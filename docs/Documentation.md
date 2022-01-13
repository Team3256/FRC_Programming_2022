
# Documentation

## Logging
In order to log a message, first initialize the logger for each class:
```java
private static Logger logger = new Logger.getLogger(MyClass.class.toString());
```
Where **MyClass is where the logger is being initialized.**

To Log Messages Use `logger.info("Message to be logged")`

The Various Logging Levels are:

**SEVERE** messages should describe events that are of considerable importance and which will prevent normal program execution.

**WARNING** messages should describe events which indicate potential problems.

**INFO** messages should only be used for reasonably significant messages.

**CONFIG** messages are intended to provide a variety of static configuration information, to assist in debugging problems that may be associated with particular configurations.

All of **FINE, FINER,** and **FINEST** are intended for relatively detailed tracing and should not be pushed to the main code base.
