/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import TrcCommonLib.trclib.TrcAutoTask;


public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = "TaskAutoPickup";

    public enum State
    {
        START,
        LOOK_FOR_TARGET,
        DRIVE_TO_TARGET,
        APPROACH_OBJECT,
        INTAKE_OBJECT,
        PICKUP_OBJECT,
        DONE
    }   //enum State

    public enum ObjectType
    {
        CUBE, 
        CONE
    }   //enum ObjectType
    
    protected void acquireSubsystemsOwnership() 
    {

    }

    protected void releaseSubsystemsOwnership()
    {

    }

    protected void stopSubsystems()
    {
        //stop stuff ig
    }

    private void runTaskState(State state)
    {


        switch(state)
        {
            case START:
                break;

            case LOOK_FOR_TARGET:
                break;
            
            case DRIVE_TO_TARGET:
                break;
            
            case APPROACH_OBJECT:
                break;
            
            case INTAKE_OBJECT:
                break;
            
            case PICKUP_OBJECT:
                break;
            
            case DONE:
                stopAutoTask(true);
                break;
        }
    }
}
