#ifndef CHARGER_H
#define CHARGER_H

typedef enum { CHARGER_UNKNOWN, CHARGER_CHARGING, CHARGER_FULL, CHARGER_FAULT } ChargerStatus_t;

void            charger_init(void);
ChargerStatus_t charger_get_status(void);

#endif /* CHARGER_H */
