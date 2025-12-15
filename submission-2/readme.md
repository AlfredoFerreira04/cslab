# CSLAB - Second Submission

## Requirement Specification

//TODO

## Implementation

### Concurrency and RT scheduling parameters

| Task                                         | C/ms | T/ms | D/ms |
|----------------------------------------------|------|------|------|
| Read and transmit data from heat sensor.     ||||
| Read and transmit data from light sensor.    ||||
| Change intensity on artificial lights        ||||
| Change position of blinds                    ||||
| Change temperature on heater (LED moq)       ||||

### Assembly Functionality

* The computation of the difference between sensor readings and the verification of whether it exceeds the acceptable limit will be implemented in assembly.

* This function takes three decimal parameters, value1, value2, and limit, and returns an integer indicating whether the limit is violated: a return value of 1 signifies that the limit was exceeded, while 0 indicates the difference remains within the acceptable range.

```assembly
//TODO
```

### System Implementation

#### Workers

//TODO

#### Server

//TODO

#### Dashboard/UI

//TODO
