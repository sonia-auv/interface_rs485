# SLAVE ID

uint8 SLAVE_powersupply0 = 48
uint8 SLAVE_powersupply1 = 49
uint8 SLAVE_killMission = 32
uint8 SLAVE_ISI_1 = 4
uint8 SLAVE_ISI_2 = 4

#CMD
# les define de la kill mission switch
uint8 CMD_MISSION  		    =0
uint8 CMD_KILL	  		    =1
# les define de la kill mission switch
uint8 CMD_PS_V16_1  		=0
uint8 CMD_PS_V16_2	     	=1
uint8 CMD_PS_V12	  		=2
uint8 CMD_PS_C16_1	    	=3
uint8 CMD_PS_C16_2	  	    =4
uint8 CMD_PS_C12	  		=5
uint8 CMD_PS_temperature	=6
uint8 CMD_PS_VBatt		    =7


uint8 CMD_ISI_power = 2
uint8 CMD_POWERSUPPLY_action1 = 2

# SERVICE DEFINITION

uint8 slave
uint8 cmd
uint8[] data