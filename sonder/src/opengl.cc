#include <sonder/opengl.hh>

#if defined(__unix__)

#include <pthread.h>
void* simpleFunc(void*) {
  return NULL;
}
void forcePThreadLink() {
  pthread_t t1;
  pthread_create(&t1, NULL, &simpleFunc, NULL);
}
#endif