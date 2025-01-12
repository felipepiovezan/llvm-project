#include <stdio.h>

void foo() {
    puts("stop here"); // Set breakpoint here
    puts("more code to step out from");
}

int main (int argc, char const *argv[], char const *envp[])
{
    foo();
    return 0;
}
