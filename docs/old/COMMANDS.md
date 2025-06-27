## PID Commands

### GET_PID [LEFT|RIGHT]
Returns current PID values in format:
`KP:x.xx KI:x.xx KD:x.xx MIN:x.xx MAX:x.xx ILIM:x.xx`

### SET_PID [LEFT|RIGHT] [PARAMS]
Set PID values. Parameters:
- KP=value (float)
- KI=value (float) 
- KD=value (float)
- MIN=value (float)
- MAX=value (float)
- ILIM=value (float)

Example:
`SET_PID LEFT KP=0.50 KI=0.10 KD=0.20 MIN=-1.0 MAX=1.0 ILIM=5.0`
