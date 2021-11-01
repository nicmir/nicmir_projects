int eeprom_read(int a_address, int a_length);
int eeprom_write(int a_address, int a_length, uint8_t *a_pData);
int config_restore();
int config_save();
int config_modify(stLFConfiguration *a_pConfig);
int config_show();
