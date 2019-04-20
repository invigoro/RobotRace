#NOT compilable
for loop, get current frame
    if pink line in frame:
        if orientation is OK:
            if line is in bottom of frame:
                move forward
                break
            else:
                move forward
        elif line is tilted down right:
            turn right
        elif line is tilted down left:
            turn left
    elif white line in frame:
        turn
    else:
        move forward

for loop, get current frame:
    if orange in frame:
        if white NOT in center of frame:
            if orientation of orange is OK:
                if line is in bottom of frame:
                    move forward
                    break
                else:
                    move forward
            elif line is tilted down right:
                turn right
            elif line is tilted down left:
                turn left
        elif white in center:
            find center of mass of white
            if COM is more right:
                turn left
            elif COM is more left:
                turn right
            else:
                turn at random I guess
    elif pink in frame:
        turn around
    elif white in frame: 
        turn
    else:
        move forward
    
for loop, get current frame:
    search for faces
    move to face
    hold out hand

for loop, get current frame:
    if(ice in hand is right color):
        close hand
        nod head
        speak
        break
    elif(ice in hand is wrong color):
        shake head

repeat first for loop with orange rather than pink

repeat second for loop with pink rather than orange

for loop, get current frame:
    if pink (bin) in frame: 
        