/*
 * can_sender.c
 *
 *  Created on: Jun 18, 2025
 *      Author: Miriam Vitolo
 */

#include "can_sender.h"
#include "queue.h"

/**
 * @brief Numero massimo di messaggi inviati in una singola chiamata a
 *        can_sender_dequeue_msg, pari al numero di mailbox TX hardware (STM32
 *        bxCAN). Funge da bound esplicito del ciclo di invio: garantisce
 *        l'uscita dal while anche se il livello di mailbox libere venisse
 *        riportato in modo errato dalla HAL.
 */
#define CAN_SENDER_MAX_TX_BURST                 (3U)

CANSender_StatusTypeDef can_sender_init(can_sender_t *can_sender, canManager_t *can_manager, osMessageQId xQueue){
	 CANSender_StatusTypeDef status = CAN_SENDER_ERROR;
	 if(can_sender != NULL && can_manager != NULL && xQueue != NULL){
		 can_sender->can_manager = can_manager;
		 can_sender->xQueue = xQueue;
		 status = CAN_SENDER_OK;
	 }

	 return status;
}

CANSender_StatusTypeDef can_sender_enqueue_msg(can_sender_t *can_sender, const can_msg_t *msg){
	 CANSender_StatusTypeDef status = CAN_SENDER_ERROR;
	 if(can_sender != NULL){
		 if (xQueueSend(can_sender->xQueue, msg, 0U) == pdPASS) {
			 status = CAN_SENDER_OK;
		  }
	 }

	 return status;
}

//TODO: qui c'è un bug: perché la canManager_SendMessage per come è implementata può dare OK anche se non è riuscita a inviare il messaggio (perché non c'erano mailboxes libere), causando l'eliminazione del messaggio dalla coda anche se non è stato inviato. Per risolvere questo problema, si potrebbe modificare la funzione canManager_SendMessage in modo che restituisca un codice di errore specifico quando non ci sono mailboxes libere, e in quel caso non rimuovere il messaggio dalla coda.
CANSender_StatusTypeDef can_sender_dequeue_msg(can_sender_t *can_sender) {
    CANSender_StatusTypeDef status = CAN_SENDER_ERROR;
    if (can_sender != NULL) {
        if (uxQueueMessagesWaiting(can_sender->xQueue) > 0U) {
            if (HAL_CAN_GetTxMailboxesFreeLevel(can_sender->can_manager->config->hcan) > 0U) {
                can_msg_t *can_msg_buff = &can_sender->can_msg_buff;
                uint32_t tx_count = 0U;
                canManager_Status_TypeDef can_manager_status = CAN_MANAGER_OK;
                while ((tx_count < CAN_SENDER_MAX_TX_BURST) &&
                       (HAL_CAN_GetTxMailboxesFreeLevel(can_sender->can_manager->config->hcan) > 0U) &&
                       (uxQueueMessagesWaiting(can_sender->xQueue) > 0U) &&
                       (xQueuePeek(can_sender->xQueue, can_msg_buff, 0U) == pdPASS) &&
                       (can_manager_status == CAN_MANAGER_OK)) {
                    can_manager_status = canManager_SendMessage(can_sender->can_manager, can_msg_buff->id, can_msg_buff->msg);
                    if ((can_manager_status == CAN_MANAGER_OK) &&
                        (xQueueReceive(can_sender->xQueue, can_msg_buff, 0U) == pdPASS)) {
                        status = CAN_SENDER_OK;
                        tx_count++;
                    } else {
                        status = CAN_SENDER_ERROR;
                        can_manager_status = CAN_MANAGER_ERR; // Exit the loop if sending fails
                    }
                }
            }
        } else {
            /* Queue empty: nothing to send, but it's not an error */
            status = CAN_SENDER_OK;
        }
    }

    return status;
}

