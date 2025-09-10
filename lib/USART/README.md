# Very simple USART library
Only 8 data bits, no parity, 1 stop bit. Other parameters still have to be implemented

## To install USART library in $HOME/Arduino:
```
cmake -B build
cmake --build build
cmake --install build
```

## Compile example (echo's only digits)
```
cmake -B build
cmake --build build
cmake --build build --target upload build
```
