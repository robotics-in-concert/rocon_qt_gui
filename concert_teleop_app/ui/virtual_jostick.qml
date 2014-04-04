import QtQuick 1.0

Item {
    id:joyStick;
    property int offset:30;
    property string direction:"C";
    property bool isPressedHover:false;
    
    signal pressedHoverChanged(bool ishover);
    signal dirChanged(string direction);

    Rectangle {
        id:totalArea
        color:"gray"
        radius: parent.width/2
        opacity: 0.5
        width:parent.width
        height:parent.height
    }

    Rectangle{
        id:stick
        width:totalArea.width/2
        height: totalArea.height/2
        radius: height/2
        x: totalArea.width/4;
        y: totalArea.height/4;        
        color:"black"
    }

    MouseArea{
        id:mouseArea
        anchors.fill: parent
        
        onMousePositionChanged: {            
         var rad = totalArea.radius;
         rad =  rad * rad;

         // calculate distance in x direction
         var xDist = mouseX - (totalArea.x + totalArea.radius);
         xDist = xDist * xDist;

         // calculate distance in y direction
         var yDist = mouseY - (totalArea.y + totalArea.radius);
         yDist = yDist * yDist;

         //total distance for inner circle
         var dist = xDist + yDist;            

         //if distance if less then radius then inner circle is inside larger circle
         if( rad < dist) {
             return;
         }

         //center of larger circle
         var oldX = stick.x; var oldY = stick.y;
         stick.x = mouseX - stick.radius;
         stick.y = mouseY - stick.radius;

         //using L R U D LU RU LD RD for describe direction
         var dir="";

         //check if Right or left direction, 
         //by checking if inner circle's y is near center of larger circle
         
         if( stick.y >= totalArea.radius - stick.radius - joyStick.offset 
             && stick.y+stick.height <= totalArea.radius + stick.radius + joyStick.offset) 
         {
             if( stick.x + stick.radius > totalArea.x + totalArea.radius) 
             {
                 dir = "R";
             } 
             else if( stick.x < totalArea.x + totalArea.radius) 
             {
                 dir = "L";
             }
         }
         
         //check if Up or Down direction, 
         //by checking if inner circle's x is near center of larger circle
         else if( stick.x >= totalArea.radius - stick.radius - joyStick.offset 
                 && stick.x + stick.width <= totalArea.radius + stick.radius + joyStick.offset)
         {
            if( stick.y + stick.radius > totalArea.y + totalArea.radius) 
            {
                 dir = "D";
            } 
            else if( stick.y < totalArea.y + totalArea.radius) {
                 dir = "U";
            }
         }
         
         //check if Up Left or Up Right direction,
         //by checking if inner circle is near one of top corner of larger circle
         
         else if( stick.y < totalArea.radius - stick.radius ) 
         {
            if( stick.x + stick.radius > totalArea.x + totalArea.radius) 
            {
                dir = "R";
            } 
            else if( stick.x < totalArea.x + totalArea.radius) 
            {
                dir = "L";
            }
            dir = dir+"U";
         }
         
         //check if Down Left or Down Right direction,
         //by checking if inner circle is near one of bottom corner of larger circle
         else if ( stick.y + stick.radius >= totalArea.radius + stick.radius ) 
         {
            if( stick.x + stick.radius > totalArea.x + totalArea.radius) 
            {
               dir = "R";
            } 
            else if( stick.x < totalArea.x + totalArea.radius) 
            {
               dir = "L";
            }
            dir = dir+"D";
         }
          joyStick.dirChanged(dir);
          joyStick.direction = dir;
          
        }
       
        onPressed: {
           joyStick.isPressedHover = true;
           pressedHoverChanged(joyStick.isPressedHover)
        }

        onReleased: {
            //snap to center
            stick.x = totalArea.width /4;
            stick.y = totalArea.height/4;
            joyStick.isPressedHover = false;
            pressedHoverChanged(joyStick.isPressedHover)
        }
    }
}
