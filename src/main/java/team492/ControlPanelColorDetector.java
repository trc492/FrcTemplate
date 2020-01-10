/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frclib.FrcRevColorSensorV3;
import trclib.TrcHashMap;

public class ControlPanelColorDetector extends FrcRevColorSensorV3
{
    private static final String moduleName = "ControlPanelColorDetector";
    private static final I2C.Port DEF_I2C_PORT = I2C.Port.kOnboard;
    private static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    private static final TrcHashMap<Color, String> colorMap = new TrcHashMap<Color, String>()
        .add(kBlueTarget, "Blue")
        .add(kGreenTarget, "Green")
        .add(kRedTarget, "Red")
        .add(kYellowTarget, "Yellow");

    public ControlPanelColorDetector(I2C.Port i2cPort)
    {
        super(moduleName, i2cPort);
        addColorMatch(kBlueTarget);
        addColorMatch(kGreenTarget);
        addColorMatch(kRedTarget);
        addColorMatch(kYellowTarget);
    }   //ControlPanelColorDetector

    public ControlPanelColorDetector()
    {
        this(DEF_I2C_PORT);
    }   //ControlPanelColorDetector

    public String getMatchedColorName()
    {
        return colorMap.get(getMatchedColor());
    }   //getMatchedColorName

}   //class ControlPanelColorDetector
