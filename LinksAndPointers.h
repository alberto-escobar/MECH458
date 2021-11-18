typedef struct {
	int itemCode; 	/* stores a number describing the element */
	char stage; 	/* 0: part is built, 1: part not built, 2: part is shipped */
} element;

typedef struct {
	element e;
	struct link *next;
} link;

/* subroutine headers */
void setup();
void clear();
void enqueue(element e);
element dequeue();
element firstValue();
char isEmpty();
int size();
