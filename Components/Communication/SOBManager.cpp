
#include "SOBManager.hpp"

bool SOBManager::IsLineAvailable()
{
	return isControllingLine && !pendingTxQ_->GetQueueMessageCount();
}

void SOBManager::ConfirmEOF()
{
    isControllingLine = true; //PI can send to the SOB when true
    // If we have a pending message, forward it to the UART task
    Command txCm;
    while (pendingTxQ_->Receive(txCm))
    {
    	HAL_UART_Transmit(SystemHandles::UART_SOB, txCm.GetDataPointer(), txCm.GetDataSize(), DEBUG_SEND_MAX_TIME_MS);

    }
}

// --  Called by UART Task on receiving a SOB Transmit Request
void SOBManager::HandleSOBTx(Command& cm)
{
    if (IsLineAvailable())
    {
    	HAL_UART_Transmit(SystemHandles::UART_SOB, cm.GetDataPointer(), cm.GetDataSize(), DEBUG_SEND_MAX_TIME_MS);
    	isControllingLine = false; // PI cannot listen to the SOB
    }
    else
    {
        // Unable to transmit, try to add it to the queue
    	pendingTxQ_->SendWait(cm);
    }
}


