int add_one(int x) { return x + 1; }
int add_two(int x) { return x + 2; }

int (*global_fp)(int) = add_one;

int main() {
  int (*local_fp)(int) = add_two;
  return global_fp(1) + local_fp(2); // break here
}
