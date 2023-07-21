

#include "cmsis_os.h"
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

protected:

	bool IsLineAvailable();
	void ConfirmEOF();
	void HandleSOBTx(Command& cm);


private:

	Queue* pendingTxQ_;
	bool isControllingLine;
	SOBManager() {}	// Private constructor
	SOBManager (const SOBManager&);						// Prevent copy-construction
	SOBManager& operator=(const SOBManager&);			// Prevent assignment
};
