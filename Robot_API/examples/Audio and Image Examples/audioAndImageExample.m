%% Initialize
robot = raspbot('sim');

%% Take a Picture
% Stores Matlab-formatted rgb image data in 'image' and displays it
image = robot.captureImage;


%% Use Text-to-Speech
% Text-to-speech input must be a string, but can contain numerals
num = 42;
toSay = 'The answer to life, the universe, and everything is '  + num2str(num);

robot.say(toSay);

%% Save an Audio File to the Robot
% sendSoundFile only works with .wav files
% The sent sound file is saved to the robot under the name fileName
localPath = 'testSounds/wavFile.wav';
remotePath = 'newSound.wav';

robot.sendSoundFile(localPath, remotePath);

%% Play a Saved Audio File
% The file must have been saved to the robot using sendSoundFile
% You must reference the file using the same name as you used to save it
robot.playSound(fileName);

%% Clean Up
robot.shutdown();
 