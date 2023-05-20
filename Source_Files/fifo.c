/*
 * fifo.c
 *
 *  Created on: Mar 1, 2022
 *      Author: Noah Schwartz
 */


#include "fifo.h"
#include "os.h"
#include "em_emu.h"
#include "stdlib.h"

struct bt_node * create_queue(int state)
{
  struct bt_node * head = malloc(sizeof(struct bt_node));
  head->next = NULL;
  head->isOn = state;
  return head;
}

struct bt_node * create_new_node(int state, struct bt_node * queue)
{
  if(queue == NULL)
    {
      return NULL;
    }
  struct bt_node * newNode = malloc(sizeof(struct bt_node));
  newNode->next = NULL;
  newNode->isOn = state;
  return newNode;
}

int pop(struct bt_node *head)
{
  if(head == NULL)
    {
      return -10;
    }
  struct bt_node * temp = head;
  int return_st = head->isOn;
  head = head->next;
  free(temp);
  return return_st;
}
struct bt_node * push(struct bt_node * head, int state)
{
  if(head == NULL)
    {
      struct bt_node * newNode = NULL;
      newNode->isOn = state;
      newNode->next = NULL;
      head = newNode;
      return head;
    }
  struct bt_node * temp = head;
  while(temp->next != NULL)
    {
      temp = temp->next;
    }
  temp->next = create_new_node(state, temp);
  temp->next->next = NULL;
  return head;
}

int is_empty(struct bt_node * head)
{
  if(head == NULL)
    {
      return 0;
    }
  else
    {
      return 1;
    }
}

