        NAME Delay

        PUBLIC delayMicroseconds
        PUBLIC delayMilliseconds
        
          CFI Names cfiNames0
          CFI StackFrame CFA R13 DATA
          CFI Resource R0:32, R1:32, R2:32, R3:32, R4:32, R5:32, R6:32, R7:32
          CFI Resource R8:32, R9:32, R10:32, R11:32, R12:32, R13:32, R14:32
          CFI EndNames cfiNames0
        
          CFI Common cfiCommon0 Using cfiNames0
          CFI CodeAlign 2
          CFI DataAlign 4
          CFI ReturnAddress R14 CODE
          CFI CFA R13+0
          CFI R0 Undefined
          CFI R1 Undefined
          CFI R2 Undefined
          CFI R3 Undefined
          CFI R4 SameValue
          CFI R5 SameValue
          CFI R6 SameValue
          CFI R7 SameValue
          CFI R8 SameValue
          CFI R9 SameValue
          CFI R10 SameValue
          CFI R11 SameValue
          CFI R12 Undefined
          CFI R14 SameValue
          CFI EndCommon cfiCommon0


        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function delayMicroseconds
          CFI NoCalls
        THUMB

delayMicroseconds:
        PUSH    {R1} //Save register state as we do not know what is in there
        MOV     R1, #14 
        MUL     R0, R1, R0

        MOV     R1, #36
        SDIV    R1, R0, R1
        ADD     R0, R0, R1

        CMP     R0, #200
        BGE     ?Subroutine0
        NOP
        NOP
        SUBS    R0, #3
        B.N     ?Subroutine0

          CFI EndBlock cfiBlock0

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function delayMilliseconds
          CFI NoCalls
        THUMB

delayMilliseconds:
        PUSH {R1} //Save register state as we do not know what is in there
        MOV R1, #+14400
        MUL R0, R1, R0
        //Fall through to ?Subroutine0 
        
          CFI EndBlock cfiBlock1
        REQUIRE ?Subroutine0

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI NoFunction
        THUMB
?Subroutine0:
        SUBS     R0, #+1
        BHI      ?Subroutine0
        POP      {R1} //Restore register to its previous state
        BX       LR               ;; return
          CFI EndBlock cfiBlock2

        END

