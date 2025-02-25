# Campion Robotics - Team 16075

For all 

Please see below some useful information:

## How to Tune PIDF Values (For Beginners)

### 1. Understand PIDF:

- **P (Proportional):** Controls response to error. Start low and increase until oscillation starts,
  then back off slightly.
- **I (Integral):** Corrects accumulated error over time. Increase it if the system struggles to
  reach the target, but don't overdo it, as it can cause instability.
- **D (Derivative):** Dampens changes. Helps smooth the response. Start with zero and slowly
  increase until the system becomes stable.
- **F (Feedforward):** Directly scales the target value. Use it to offset known constant forces (
  e.g., gravity, friction). This is usually calculated based on system modeling.

### 2. Steps to Tune:

#### a. Start with only P (set I, D, F to 0):

- Gradually increase **P** until the system starts oscillating around the target.
- Once it oscillates, reduce **P** slightly until it stabilizes.

#### b. Add I:

- Slowly increase **I** to help the system reach the target accurately.
- Stop when you observe minimal overshooting or oscillation.

#### c. Add D:

- Increase **D** to reduce overshoot and stabilize response further.
- Too much **D** can make the system sluggish, so balance is key.

#### d. Adjust F (if applicable):

- Calculate or estimate the **F** value based on known forces acting on the system.
- Use this as a starting point and fine-tune

## OpCode Naming Convention

When naming our OpCodes, we try to follow a concept called **functionally explicit nomenclature**.
What does this mean? This means that each OpCode's name should clearly convey its purpose and
functionality without requiring additional context or explanation. The goal is to make the code more
easily traversable by ensuring that the purpose of each OpMode is immediately apparent
to developers, even those who may not be familiar with the specific implementation details. For
example,
OpModes should follow this format:

```
Classname:
[Alliance][Auto/Tele][Function/Description]

Name:
[Alliance] | [Auto/Tele] - [Driver|N/A] | [Function/Description]
```

e.g. RedAutoMain
Red | Auto - N/A | Main

BlueTeleMain
Blue | Auto - Rithvik | Main

Each OpMode sample class begins with several lines of code like the ones shown below:

### How to set and name OpCodes

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.