#include <iostream>

int addNumbers(int a, int b) {
    return a + b;
}

int main() {
    int num1 = 5;
    int num2 = 10;
    int sum = addNumbers(num1, num2);
    std::cout << "Sum: " << sum << std::endl;
    return 0;
}