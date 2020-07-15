#
# @author Lorenzo Gentilini (University of Bologna)
# May, 2020
#

px4_add_module(
	MODULE modules__extremum_seeking
	MAIN extremum_seeking
	STACK_MAIN 1000
	SRCS
		extremum_seeking.cpp
	DEPENDS
	)
