.globl unf_covar
.globl unf_var
.globl unf_best_guess
.globl unf_create
.globl unf_create_exact

.globl sqrtunf
.globl asinunf
.globl atan2unf

unf_covar:
	ungcov.s fa0, fa0, fa1
	ret

unf_var:
	unfmv.s fa1, fa0
	tail unf_covar

unf_best_guess:
	ret

unf_create:
	unsvar.s fa0, fa1
	ret

unf_create_exact:
	unclvar.s fa0
	ret

/* UNCERTAINTY AWARE MATHEMATICS
 *
 * Note that these functions use the same uncertain and standard memory addresses
 * for all loads and stores. This matches the assembly produced by the
 * gcc -> python script -> as -> python script build set up.
 */

sqrtunf:
	unfsqrt.s fa0, fa0
	ret

atan2unf:
	/* Prologue */

	addi      sp, sp, -16
	sw        ra, 12(sp)
	unfsw     fs0, 8(sp), 8(sp)
	unfsw     fs1, 4(sp), 4(sp)

	/* Procedure body */

	/* Copy y and x into fs0 to preserve their values during procedure call. */
	unfmv.s   fs0, fa0
	unfmv.s   fs1, fa1


	/* atan2f is a function of both y and x but the UNUPG.S instruction
	 * can only be used on functions of one input.
	 *
	 * However, atan2f(y, x) = atanf(y/x) + constant. (The value of the
	 * constant does depend on y and x but has a gradient of 0 everywhere
	 * and is discontinuous.)
	 *
	 * Therefore, we can calculate uncertainty information about a third
	 * variable z = y/x using uncertainty aware division and then apply the
	 * UNUPG.S instruction to atanf(z).
	 */

	/* Calculate the best guess of atan2f(y, x) and store in fa0. */
	call      atan2f

	/* Calculate z = y / x and store in fa4 */
	unfdiv.s	fa4, fs0, fs1

	/* Calculate d(atan(z))/dz = 1 / (z^2 + 1) and store in fa2. */
	li		a0, 1
	fcvt.s.w	fa3, a0
	fmul.s		fa2, fs0, fs0
	fadd.s		fa2, fa2, fa3
	fdiv.s		fa2, fa3, fa2

	/* Set the uncertainty information of fa0 by applying the linear
	 * uncertainty propagation equations to atan(z).
	 * The best guess stored in fa0 is left unchanged by UNUPG.S.
	 */
	unupg.s		fa0, fa4, fa2

	/* Epilogue */

	unflw     fs0, 8(sp), 8(sp)
	lw        ra, 12(sp)
	addi      sp, sp, 16
	ret

asinunf:
	/* Prologue */

	addi      sp, sp, -16
	sw        ra, 12(sp)
	unfsw     fs0, 8(sp), 8(sp)

	/* Procedure body */

	/* Copy x into fs0 to preserve its value during procedure call. */
	unfmv.s   fs0, fa0
	/* Calculate the best guess of asin(x) and store in fa0. */
	call      asinf

	/* Calculate d(asin(x))/dx = 1 / sqrt(1 - x^2) and store in fa4. */
	li        a0, 1
	fcvt.s.w  fa3, a0
	fmul.s    fa4, fs0, fs0
	fsub.s    fa4, fa3, fa4
	fsqrt.s   fa4, fa4
	fdiv.s    fa4, fa3, fa4

	/* Set the uncertainty information of fa0 by applying the linear
	 * uncertainty propagation equations to asin(x).
	 * The best guess stored in fa0 is left unchanged by UNUPG.S.
	 */
	unupg.s   fa0, fs0, fa4

	/* Epilogue */

	unflw     fs0, 8(sp), 8(sp)
	lw        ra, 12(sp)
	addi      sp, sp, 16
	ret
