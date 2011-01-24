#!/usr/bin/perl

use Config::IniFiles;

while ( my $arg = shift @ARGV ) {

	tie %inidata, 'Config::IniFiles', (-file => $arg );
	$ini = Config::IniFiles->new( -file => $arg );
	
	print "<?xml version=\"1.0\"?>\n";
	print "<config>\n";
	foreach($ini->Sections) {
		$section = $_;
		@comment = $ini->GetSectionComment($section);
		if (@comment) {
			print "<!--\n";
			foreach (@comment) {
				print "$_\n";
			};
			print "-->\n";
		}
	
		print "<$section>\n";
		foreach ($ini->Parameters($section)) {
			print "\t<$_>$inidata{$section}{$_}</$_>\n";
		}
		print "</$section>\n";
	}
	print "</config>\n";

}
