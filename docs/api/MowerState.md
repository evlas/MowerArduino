[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# MowerState Class

## Methods

### Quick Reference

- [virtual void enter(Mower& mower)](#enter)
- [virtual void exit(Mower& mower)](#exit)
- [virtual const char* getName()](#getname)
- [virtual State getStateType()](#getstatetype)
- [virtual void handleError(Mower& mower, const char* errorMsg)](#handleerror)
- [virtual void handleEvent(Mower& mower, Event event)](#handleevent)
- [virtual void update(Mower& mower)](#update)

### Method Details

#### enter

```cpp
virtual void enter(Mower& mower)
```

**Parameters:**

- `mower` (Mower&): No description

**Returns:**

virtual void - 

---

#### exit

```cpp
virtual void exit(Mower& mower)
```

**Parameters:**

- `mower` (Mower&): No description

**Returns:**

virtual void - 

---

#### getName

```cpp
virtual const char* getName()
```

**Returns:**

virtual const char* - 

---

#### getStateType

```cpp
virtual State getStateType()
```

**Returns:**

virtual State - 

---

#### handleError

```cpp
virtual void handleError(Mower& mower, const char* errorMsg)
```

**Parameters:**

- `mower` (Mower&): No description
- `errorMsg` (const char*): No description

**Returns:**

virtual void - 

---

#### handleEvent

```cpp
virtual void handleEvent(Mower& mower, Event event)
```

**Parameters:**

- `mower` (Mower&): No description
- `event` (Event): No description

**Returns:**

virtual void - 

---

#### update

```cpp
virtual void update(Mower& mower)
```

**Parameters:**

- `mower` (Mower&): No description

**Returns:**

virtual void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
