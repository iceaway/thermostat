#ifndef __RBUF_H
#define __RBUF_H

#define RBUF_SIZE 32

struct rbuf {
  char buffer[RBUF_SIZE];
  volatile int head;
  volatile int tail;
  int count;
};

void rbuf_init(struct rbuf *rb);
int rbuf_pop(struct rbuf *rb, char *data);
int rbuf_push(struct rbuf *rb, char data);
int rbuf_empty(struct rbuf *rb);

#endif
