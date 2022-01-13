
# Logging

### Usage
#### Initialization
```java
private static final Logger logger = Logger.getLogger(MyClass.class.getCanonicalName());
```
Where **MyClass is where the logger is being initialized.**

#### Normal Logging

To Log Messages Use `logger.info("Message to be logged");`

#### Logging Exceptions

To log exceptions you should use this notation
```java
try { 
  Code that Generates Exception 
} catch (IOException e){
  logger.log(Level.WARNING, e.getMessage(), e);
}
```
Of course, Changing the Level to something appropriate. Such as `Level.SEVERE`

### Various Logging Levels are:

**SEVERE** messages should describe events that are of considerable importance and which will prevent normal program execution.

**WARNING** messages should describe events which indicate potential problems.

**INFO** messages should only be used for reasonably significant messages.

**CONFIG** messages are intended to provide a variety of static configuration information, to assist in debugging problems that may be associated with particular configurations.

All of **FINE, FINER,** and **FINEST** are intended for relatively detailed tracing and should **not** be pushed to the main code base.
