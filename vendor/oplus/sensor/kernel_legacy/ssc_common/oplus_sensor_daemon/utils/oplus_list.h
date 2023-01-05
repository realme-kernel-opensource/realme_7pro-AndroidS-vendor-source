#ifndef __LIST_H
#define __LIST_H
#include <stdio.h>

#ifndef offsetof
#define offsetof(type, md) ((unsigned long)&((type *)0)->md)
#endif

#ifndef container_of
#define container_of(ptr, type, member) \
  ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

struct list_head {
    struct list_head *next, *prev;
};

static inline void INIT_LIST_HEAD(struct list_head *list)
{
    list->next = list;
    list->prev = list;
}

static inline void __list_add(struct list_head *new,
    struct list_head *prev,
    struct list_head *next)
{
    next->prev = new;
    new->next = next;
    new->prev = prev;
    prev->next = new;
}

static inline void list_add(struct list_head *new, struct list_head *head)
{
    __list_add(new, head, head->next);
}

static inline void list_add_tail(struct list_head *new, struct list_head *head)
{
    __list_add(new, head->prev, head);
}

static inline void __list_del(struct list_head * prev, struct list_head * next)
{
    next->prev = prev;
    prev->next = next;
}

static inline void __list_del_entry(struct list_head *entry)
{

    __list_del(entry->prev, entry->next);
}

static inline void list_del(struct list_head *entry)
{
    __list_del_entry(entry);
    entry->next = NULL;
    entry->prev = NULL;
}

static inline int list_is_last(const struct list_head *list,
    const struct list_head *head)
{
    return list->next == head;
}

static inline int list_empty(const struct list_head *head)
{
    return ((head->next) == head);
}

#define list_entry(ptr, type, member) \
	container_of(ptr, type, member)

#define list_first_entry(ptr, type, member) \
	list_entry((ptr)->next, type, member)

#define list_next_entry(pos, type, member) \
	list_entry((pos)->member.next, type , member)

#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

#define list_for_each_entry(pos, type, head, member)				\
	for (pos = list_first_entry(head, type, member);	\
		&pos->member != (head);					\
		pos = list_next_entry(pos, type, member))

#define list_for_each_entry_safe(pos, n, type, head, member)			\
	for (pos = list_first_entry(head, type, member),	\
		n = list_next_entry(pos, type, member);			\
	     &pos->member != (head);					\
	     pos = n, n = list_next_entry(n, type, member))

#endif //__LIST_H
