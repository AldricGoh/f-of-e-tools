	.file	"worst_branch.c"
	.option nopic
	.text
	.align	2
	.globl	main
	.type	main, @function
main:
	addi	sp,sp,-32
	sw	s0,28(sp)
	addi	s0,sp,32
	li	a5,8192
	sw	a5,-28(s0)
	sw	zero,-32(s0)
	lw	a5,-28(s0)
	lw	a5,0(a5)
	not	a4,a5
	lw	a5,-28(s0)
	sw	a4,0(a5)
	sw	zero,-20(s0)
	j	.L2
.L5:
	sw	zero,-24(s0)
	j	.L3
.L4:
	lw	a5,-24(s0)
	addi	a5,a5,1
	sw	a5,-24(s0)
.L3:
	lw	a4,-24(s0)
	lw	a5,-32(s0)
	blt	a4,a5,.L4
	lw	a5,-20(s0)
	addi	a5,a5,1
	sw	a5,-20(s0)
.L2:
	lw	a4,-20(s0)
	li	a5,401408
	addi	a5,a5,-1409
	ble	a4,a5,.L5
	li	a5,0
	mv	a0,a5
	lw	s0,28(sp)
	addi	sp,sp,32
	jr	ra
	.size	main, .-main
	.ident	"GCC: (GNU) 8.2.0"
