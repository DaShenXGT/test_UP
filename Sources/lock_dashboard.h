#ifndef	_LOCK_H
#define	_LOCK_H

typedef   struct 
{
	unsigned char		cmd_id;								//
	unsigned char		lock_status;					
	
}vehicle_lock;

extern	vehicle_lock	munich_lock;
void	Lock_Handle(void);

#endif
