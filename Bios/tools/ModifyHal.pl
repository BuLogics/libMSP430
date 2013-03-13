################################################################################
#!/win32/perl/bin/perl                                                         #
#------------------------------------------------------------------------------#
#                                                                              #
#  Author:     Patrick de Cesar Francisco                                      #
#                                                                              #
#                                                                              #
# Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/       #
#                                                                              #
#                                                                              #
#  Redistribution and use in source and binary forms, with or without          #
#  modification, are permitted provided that the following conditions          #
#  are met:                                                                    #
#                                                                              #
#    Redistributions of source code must retain the above copyright            #
#    notice, this list of conditions and the following disclaimer.             #
#                                                                              #
#    Redistributions in binary form must reproduce the above copyright         #
#    notice, this list of conditions and the following disclaimer in the       #
#    documentation and/or other materials provided with the                    #
#    distribution.                                                             #
#                                                                              #
#    Neither the name of Texas Instruments Incorporated nor the names of       #
#    its contributors may be used to endorse or promote products derived       #
#    from this software without specific prior written permission.             #
#                                                                              #
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         #
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT           #
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR       #
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT        #
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,       #
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT            #
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,       #
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY       #
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT         #
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE       #
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.        #
#                                                                              #
#------------------------------------------------------------------------------#
# Description: Create new directories.  									   #
#			   Change the data inside files  			  	        		   #
#			   Save the new file into the new directory						   #
#------------------------------------------------------------------------------#
################################################################################

# -- Use Modules ---------------------------------------------------------------------------------------------------------
use warnings;

#public vars:

$infile = "src/hal/UifHal.h";    
#$infile =  "C:/MSP430/a0406322_V3DLL/msp430_tools/Projects/MSP430_DLLv3/Bios/src/up/core_Image_V3.h";

$output = "include/UifHal.h";
#$output = "C:/MSP430/a0406322_V3DLL/msp430_tools/Projects/MSP430_DLLv3/Bios/include/Core.h";
    
    
# -- Variables   --------------------------------------------------------------------------------------------------------
my $temp_text = "";						
my $string = "";

my $lookfor_1 = "unsigned short";
my $lookfor_variable_1 ="unsigned short" ;
my $change_variable_to_1 = "uint16_t";

my $lookfor_2 = "unsigned long";
my $lookfor_variable_2 ="unsigned long" ;
my $change_variable_to_2 = "uint32_t";

# -- Routine prototypes   -----------------------------------------------------------------------------------------------
sub replace_string;
sub main();

# -- Routines   ---------------------------------------------------------------------------------------------------------

sub replace_string_1
{
   $string = "@_";
   $string =~ s/$lookfor_variable_1/$change_variable_to_1/;
   return($string);
}

sub replace_string_2
{
   $string = "@_";
   $string =~ s/$lookfor_variable_2/$change_variable_to_2/;
   return($string);
}

sub main()
{
	open FILE, $infile or die $!;
	open OUTPUT_FILE, ">$output";

	# Read data from file
	my @lines = <FILE>;

	my $i;
	my $temp_text = '';
	
	for($i = 1; $i< @lines; $i=$i+1)
	{
		$temp_text = $lines[$i];

		# perform variable change
		if($temp_text =~ m/ ($lookfor_1) /)      
		{
			$temp_text = replace_string_1($temp_text); 
			$lines[$i] = $temp_text;
		}
		if($temp_text =~ m/ ($lookfor_2) /)      
		{
			$temp_text = replace_string_2($temp_text); 
			$lines[$i] = $temp_text;
		}
		
	}
	print OUTPUT_FILE @lines;
	
	# Save files
	close (OUTPUT_FILE);
	close (FILE);
}

eval {main()};
warn $@ if $@;

# -- END   -----------------------------------------------------------------------------------------------------------------