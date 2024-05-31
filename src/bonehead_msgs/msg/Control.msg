# stance = 0, forward walk = 1, reverse walk = 2
uint8 state

# adjust leg heights based on roll / pitch command
float32 fr_z_comp
float32 fl_z_comp
float32 br_z_comp
float32 bl_z_comp