Camera_State:
is an ENUM class for creating and choosing between only 2 modes of camera state to avoid any typos in Strings or wrong comparisons

CreateViewThread : 
class starts a new thread for the new view and contains all stitching and display functions also 
the intialization of the class is with an array of views to send a one view for normal cameras or multiple views for stitching 2 views or more

FileExplorerDialogue:
frontend of class of selecting the video path from the file explorer

New_View :
frontend class for creating a new view to be displayed later in the Camera System Window with fixed charachteristics

CameraSystemGUI :
The frontend class responisble for creating the different views and threads by calling all the other classes and displaying the end GUI
also responsibile for handling the close event of the system 
