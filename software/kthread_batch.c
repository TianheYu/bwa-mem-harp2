#include <pthread.h>
#include <stdlib.h>

extern pthread_t *worker_threads;

struct kt_for_batch_t;

typedef struct {
	struct kt_for_batch_t *t;
	int i;
} ktf_worker_batch_t;

typedef struct kt_for_batch_t {
	int n_threads, n, batch_size;
	ktf_worker_batch_t *w;
	void (*func)(void*,int,int,int);
	void *data;
} kt_for_batch_t;

static inline int steal_work(kt_for_batch_t *t)
{
	int i, k, min = 0x7fffffff, min_i = -1;
	for (i = 0; i < t->n_threads; ++i)
		if (min > t->w[i].i) min = t->w[i].i, min_i = i;
	k = __sync_fetch_and_add(&t->w[min_i].i, t->n_threads*t->batch_size);
	return k >= t->n? -1 : k;
}

static void *ktf_worker_batch(void *data)
{
	ktf_worker_batch_t *w = (ktf_worker_batch_t*)data;
	int i, batch_size;
	for (;;) {
		i = __sync_fetch_and_add(&w->i, w->t->n_threads*w->t->batch_size);
		if (i >= w->t->n) break;
		batch_size = w->t->n-i > w->t->batch_size? w->t->batch_size : w->t->n-i;
		w->t->func(w->t->data, i, batch_size, w - w->t->w);
	}
	while ((i = steal_work(w->t)) >= 0) {
		batch_size = w->t->n-i > w->t->batch_size? w->t->batch_size : w->t->n-i;
		w->t->func(w->t->data, i, batch_size, w - w->t->w);
	}
	pthread_exit(0);
}

void kt_for_batch(int n_threads, void (*func)(void*,int,int,int), void *data, int n, int batch_size)
{
	int i;
	kt_for_batch_t t;
	pthread_t *tid;
	t.func = func, t.data = data, t.n_threads = n_threads, t.n = n, t.batch_size = batch_size;
	t.w = (ktf_worker_batch_t*)alloca(n_threads * sizeof(ktf_worker_batch_t));
	tid = (pthread_t*)alloca(n_threads * sizeof(pthread_t));
	worker_threads = tid;
	for (i = 0; i < n_threads; ++i)
		t.w[i].t = &t, t.w[i].i = i*batch_size;
	for (i = 0; i < n_threads; ++i) pthread_create(&tid[i], 0, ktf_worker_batch, &t.w[i]);
	for (i = 0; i < n_threads; ++i) pthread_join(tid[i], 0);
}
