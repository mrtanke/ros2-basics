#include <iostream>
#include <algorithm>

int main()
{
    auto add = [](int a, int b) -> int
    { return a + b; };

    int sum = add(200, 50);

    auto print_sum = [sum]()
    {
        std::cout << "The sum is: " << sum << std::endl;
    };

    print_sum();
    return 0;
}