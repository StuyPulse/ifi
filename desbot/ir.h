#ifndef _IR_H
#define _IR_H

#define IR1 (rc_dig_in05)
#define IR2 (rc_dig_in06)
#define IR3 (rc_dig_in07)
#define IR4 (rc_dig_in08)

#define DRIFT_ANG 500

char check_ir(void);
void init_ir(void);
void IR_Drive(void);

#endif
