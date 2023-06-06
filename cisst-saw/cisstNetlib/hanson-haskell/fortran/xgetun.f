      SUBROUTINE XGETUN(IUNIT)                                          XER   10
C
C     ABSTRACT
C        XGETUN GETS THE (FIRST) OUTPUT FILE TO WHICH ERROR MESSAGES
C        ARE BEING SENT.  TO FIND OUT IF MORE THAN ONE FILE IS BEING
C        USED, ONE MUST USE THE XGETUA ROUTINE.
C
C     DESCRIPTION OF PARAMETER
C      --OUTPUT--
C        IUNIT - THE LOGICAL UNIT NUMBER OF THE  (FIRST) UNIT TO
C                WHICH ERROR MESSAGES ARE BEING SENT.
C                A VALUE OF ZERO MEANS THAT THE DEFAULT FILE, AS
C                DEFINED BY THE I1MACH ROUTINE, IS BEING USED.
C
C     WRITTEN BY RON JONES, WITH SLATEC COMMON MATH LIBRARY SUBCOMMITTEE
C END OF ABSTRACT
C     LATEST REVISION --- 23 MAY 1979
C
      IUNIT = J4SAVE(3,0,.FALSE.)
      RETURN
      END
