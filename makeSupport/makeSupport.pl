#!/usr/bin/perl
#
# Tool to create a new Asyn support area below the current directory
#
# Author: Andrew Johnson <anj@aps.anl.gov>
# Date: 17 May 2004
#
# $Id: makeSupport.pl,v 1.4 2004-06-22 19:29:29 norume Exp $
#

use strict;
use lib ($ENV{'EPICS_BASE'}."/lib/perl");

use Getopt::Std;
use Cwd qw(cwd abs_path);
use EPICS::Release;

# Command line options processing
our ($opt_A, $opt_B, $opt_d, $opt_h, $opt_l, $opt_t, $opt_T);
Usage() unless getopts('A:B:dhlt:T:');

# Handle the -h command
Usage() if $opt_h;

# Get some basic information
our $top = cwd();
print "\$top = $top\n" if $opt_d;

our $arch = $ENV{EPICS_HOST_ARCH};
print "\$arch = $arch\n" if $opt_d;

our $user = $ENV{USER} ||
	    $ENV{USERNAME} ||
	    Win32::LoginName()
    or die "Can't determine your username!\n";
print "\$user = $user\n" if $opt_d;

our $date = localtime;
print "\$date = $date\n" if $opt_d;

# Find asyn directory
our $asyn;
if ($opt_A) {
    $asyn = abs_path($opt_A);
} else {
    # Work it out from this script's pathname
    my @asyn = split /[\/\\]/, $0;
    warn "expected 'makeSupport.pl', got '$_'\n"
	unless 'makeSupport.pl' eq ($_ = pop @asyn);
    warn "expected 'expected $arch', got '$_'\n"
	unless $arch eq ($_ = pop @asyn);
    warn "expected 'bin', got '$_'\n"
	unless 'bin' eq ($_ = pop @asyn);
    $asyn = abs_path(join "/", @asyn);
}
print "\$asyn = $asyn\n" if $opt_d;
die "ASYN directory '$asyn' not found!\n"
    unless -d $asyn;
die "ASYN ($asyn) is missing a configure/RELEASE file!\n"
    unless -f "$asyn/configure/RELEASE";

# Find templates directory
our $templates = abs_path($opt_T ||
			 $ENV{EPICS_ASYN_TEMPLATE_DIR} ||
			 "$asyn/templates");
print "\$templates = $templates\n" if $opt_d;
die "Templates directory '$templates' does not exist\n"
    unless -d $templates;

# Handle the -l command
if ($opt_l) {
    listTemplates($templates);
    exit 0;
}

# Check the template type
our $type = $opt_t ||
	    $ENV{EPICS_ASYN_TEMPLATE_DEFAULT} ||
	    'default';
print "\$type = $type\n" if $opt_d;
print "The template '$type' does not exist.\n"
    unless $opt_l or -d "$templates/$type";

unless (-d "$templates/$type") {
    listTemplates($templates);
    exit 1;
}

# Check app name
our $name = shift or die "You didn't name your application!\n";
print "\$name = $name\n" if $opt_d;
our $lowername = lc($name);

# Find EPICS_BASE, and SNCSEQ if set;
our ($base, $sncseq);
if ($opt_B) {
    $base = abs_path($opt_B);
} else {
    my %release = (TOP => $top);
    my @apps = ('TOP');
    &readReleaseFiles("$asyn/configure/RELEASE", \%release, \@apps);
    &expandRelease(\%release);
    $base = abs_path($release{EPICS_BASE});
    $sncseq = abs_path($release{SNCSEQ}) if exists $release{SNCSEQ};
}
print "\$base = $base\n" if $opt_d;
die "EPICS_BASE directory '$base' does not exist\n"
    unless -d $base;

# Substitutions work like this:
#   (key => "val")  in the %subs hash causes
#   '_key_'  in the template to be instantiated as  'val'
# Individual templates can extend these hashes in the $type.pl script

# Substitutions used in filenames
our %namesubs = (
    NAME => $name
);

# Substitutions used in file text
our %textsubs = (
    NAME => $name,
    LOWERNAME => $lowername,
    ASYN => $asyn,
    EPICSBASE => $base,
    SNCSEQ => $sncseq,
    TOP  => $top,
    ARCH => $arch,
    TYPE => $type,
    USER => $user,
    DATE => $date
);

# Load and run any template-specific script
my $script = "$templates/$type.pl";
require $script if -f $script;

# Do it!
print "Copying ";
copyTree("$templates/top", $top, \%namesubs, \%textsubs);
copyTree("$templates/$type", $top, \%namesubs, \%textsubs) if $type ne "top";
print "\nTemplate instantiated.\n";


##### File contains subroutines only below here

sub Usage {
    print <<EOF;
Usage:
    makeSupport.pl -h
	Display help on command options
    makeSupport.pl -l [l-opts]
	List available template types
    makeSupport.pl -t type [t-opts] support
	Create tree for device 'support' from template 'type'
EOF
    print <<EOF if ($opt_h);

    where
	[l-opts] may be one of:
	    -T /path/to/templates
	    -A /path/to/asyn
	[t-opts] are any of:
	    -A /path/to/asyn
	    -B /path/to/base
	    -T /path/to/templates

Example:
    mkdir Keithley196
    cd Keithley196
    <asyn>/bin/<arch>/makeSupport.pl -t devGpib K196
EOF
    exit $opt_h ? 0 : 1;
}


# List the templates available

sub listTemplates {
    my ($templates) = @_;
    opendir TEMPLATES, $templates or die "opendir failed: $!";
    my @templates = readdir TEMPLATES;
    closedir TEMPLATES;
    print "Templates available are:\n";
    foreach (@templates) {
	next if m/^\./;
	next if m/^CVS$/;   # Shouldn't be necessary, but...
	next unless -d "$templates/$_";
	print "\t$_\n";
    }
}



# Copy directories and files from the template

sub copyTree {
    my ($src, $dst, $Rnamesubs, $Rtextsubs) = @_;
    # $Rnamesubs contains substitutions for file names,
    # $Rtextsubs contains substitutions for file content.
    
    opendir FILES, $src or die "opendir failed while copying $src: $!\n";
    my @entries = readdir FILES;
    closedir FILES;
    
    foreach (@entries) {
	next if m/^\.\.?$/;  # ignore . and ..
	next if m/^CVS$/;   # Shouldn't exist, but...
	
	my $srcName = "$src/$_";
	
	# Substitute any _VARS_ in the name
	s/_(\w+?)_/$Rnamesubs->{$1} || "_$1_"/eg;
	my $dstName = "$dst/$_";
	
	if (-d $srcName) {
	    print ":" unless $opt_d;
	    copyDir($srcName, $dstName, $Rnamesubs, $Rtextsubs);
	} elsif (-f $srcName) {
	    print "." unless $opt_d;
	    copyFile($srcName, $dstName, $Rtextsubs);
	} elsif (-l $srcName) {
	    warn "\nSoft link in template, ignored:\n\t$srcName\n";
	} else {
	    warn "\nUnknown file type in template, ignored:\n\t$srcName\n";
	}
    }
}

sub copyFile {
    my ($src, $dst, $Rtextsubs) = @_;
    return if (-e $dst);
    print "Creating file '$dst'\n" if $opt_d;
    open(SRC, "<$src") and open(DST, ">$dst") or die "$! copying $src to $dst\n";
    while (<SRC>) {
	# Substitute any _VARS_ in the text
	s/_(\w+?)_/$Rtextsubs->{$1} || "_$1_"/eg;
	print DST;
    }
    close DST;
    close SRC;
}

sub copyDir {
    my ($src, $dst, $Rnamesubs, $Rtextsubs) = @_;
    if (-e $dst && ! -d $dst) {
	warn "\nTarget exists but is not a directory, skipping:\n\t$dst\n";
	return;
    }
    print "Creating directory '$dst'\n" if $opt_d;
    mkdir $dst, 0777 or die "Can't create $dst: $!\n"
	unless -d $dst;
    copyTree($src, $dst, $Rnamesubs, $Rtextsubs);
}
