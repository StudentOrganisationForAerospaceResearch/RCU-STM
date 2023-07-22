#ifndef SOAR_SOBMANAGER_HPP_
#define SOAR_SOBMANAGER_HPP_
#include "cmsis_os.h"
#include "main.h"
#include "Queue.hpp"
#include "SystemDefines.hpp"

/* Class ------------------------------------------------------------------*/
class SOBManager
{
public:
	static SOBManager& Inst() {
		static SOBManager inst;
		return inst;
	}

	bool IsLineAvailable();
	void ConfirmEOF();
	void HandleSOBTx(Command& cm);


private:
	Queue* pendingTxQ_;
	bool isControllingLine;
	SOBManager() {pendingTxQ_ = new Queue(); isControllingLine = false;};        // Private constructor
	SOBManager (const SOBManager&);						// Prevent copy-construction
	SOBManager& operator=(const SOBManager&);			// Prevent assignment
};


#endif    // SOAR_SOBMANAGER_HPP_
