/*
 * fifo.h
 *
 *  Created on: Mar 1, 2022
 *      Author: Noah Schwartz
 */

#ifndef SRC_HEADER_FILES_FIFO_H_
#define SRC_HEADER_FILES_FIFO_H_

struct bt_node {

    int isOn; //0 = on, 1 = off
    struct bt_node* next;
};

struct bt_node * create_queue(int state);
struct bt_node * create_new_node(int state, struct bt_node * queue);
int pop(struct bt_node *head);
struct bt_node * push(struct bt_node * head, int state);
int is_empty(struct bt_node * head);

#endif /* SRC_HEADER_FILES_FIFO_H_ */
