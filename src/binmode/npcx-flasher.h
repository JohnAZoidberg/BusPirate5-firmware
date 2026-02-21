#ifndef NPCX_FLASHER_H
#define NPCX_FLASHER_H

extern const char npcx_flasher_name[];
void npcx_flasher_setup(void);
void npcx_flasher_setup_message(void);
void npcx_flasher_service(void);
void npcx_flasher_cleanup(void);

#endif
