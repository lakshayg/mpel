#ifndef MPEL_TEST_H
#define MPEL_TEST_H

#define MPEL_TEST(test_func) {                \
    printf("== Running ["#test_func"] ==\n"); \
    test_func(); printf("\n");                \
}

#endif
