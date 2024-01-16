MODULE MainModule
    ! COMMUNICATION RELATED VARIABLES
	VAR socketdev serverSocket;
    VAR socketdev clientSocket;
    VAR string data;
    
    !----------------CREATING ROUTINE----------------!
    !HOME POSITION
    CONST jointtarget CalPosR:=[[0.00150447,-130.005,29.9997,0.00387009,40.0001,-0.00486629],[-134.997,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget home:=[[0,0,0],[0.23, 0.66,-0.23,-0.66],[0,0,0,0],[0,9E+09,9E+09,9E+09,9E+09,9E+09]];
    !ALIGN TO GRIPPER AXIS POSITION
    CONST robtarget gripper_axis:=[[0.2,0.15,0.2],[0.23, 0.66,-0.23,-0.66],[0,0,0,0],[0,9E+09,9E+09,9E+09,9E+09,9E+09]];
    !ALIGN THE GRIPPER
    PERS tooldata gripper_align:=[TRUE,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

    !GRASP POINT
    CONST robtarget gripper_point:=[[0,0.3,0],[0.23, 0.66,-0.23,-0.66],[0,0,0,0],[0,9E+09,9E+09,9E+09,9E+09,9E+09]];

    !DROP POINT
	TASK PERS tooldata TGripR:=[TRUE,[[0,0,0],[1,0,0,0]],[0.262,[7.8,11.9,50.7],[1,0,0,0],0.00022,0.00024,9E-05]];
    
	PROC main()
        MoveJ home, v100, z50, TGripR;
        WaitTime 1;
		MoveJ gripper_axis, v100, z50, TGripR;
        WaitTime 1;
        MoveJ gripper_point, v100, z50, TGripR;
		!PickPlace;
		!pickplace1;
	ENDPROC
    
    ! FUNCTION FOR COMMUNICATION
    
ENDMODULE