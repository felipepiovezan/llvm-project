void callee(void) {}
void (*fp)(void) = callee;
int main(void) {
  fp();
  return 0;
}
