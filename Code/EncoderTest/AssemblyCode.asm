			ld	r30, X+					// Load indirect and post-inc
			ld	r31, X+					// Load indirect and post-inc
			ld	r24, Z					// Load indirect						// r24 = pin1 input
			ld	r30, X+					// Load indirect and post-inc
			ld	r31, X+					// Load indirect and post-inc
			ld	r25, Z					// Load indirect						// r25 = pin2 input
			ld	r30, X+					// Load indirect and post-inc			// r30 = pin1 mask
			ld	r31, X+					// Load indirect and post-inc			// r31 = pin2 mask
			ld	r22, X					// Load indirect						// r22 = state
			andi	r22, 3				// Logical AND register and constant	
			and	r24, r30				// Logical AND registers
			breq	L%=1				// Branch if equal						// if (pin1)
			ori	r22, 4					// Logical OR register and constant		//	state |= 4

		//
		L%=1:
			and	r25, r31				// Logical AND registers
			breq	L%=2				// Branch if equal						// if (pin2)
			ori	r22, 8					// Logical OR register and constant		//	state |= 8

		//
		L%=2:
			ldi	r30, lo8(pm(L%=table))	// Load immediate
			ldi	r31, hi8(pm(L%=table))	// Load immediate
			add	r30, r22				// Add two registers
			adc	r31, __zero_reg__		// Add with carry, two registers
			asr	r22						// Arithmatic shift right
			asr	r22						// Arithmatic shift right
			st	X+, r22		  			// Store indirect						// store new state
			ld	r22, X+					// Load indirect and post-inc
			ld	r23, X+					// Load indirect and post-inc
			ld	r24, X+					// Load indirect and post-inc
			ld	r25, X+					// Load indirect and post-inc
			ijmp						// Indirect jump to Z

		//
		L%=table:				
			rjmp	L%=end				// Relative jump						// 0
			rjmp	L%=plus1			// Relative jump						// 1
			rjmp	L%=minus1			// Relative jump						// 2
			rjmp	L%=plus2			// Relative jump						// 3
			rjmp	L%=minus1			// Relative jump						// 4
			rjmp	L%=end				// Relative jump						// 5
			rjmp	L%=minus2			// Relative jump						// 6
			rjmp	L%=plus1			// Relative jump						// 7
			rjmp	L%=plus1			// Relative jump						// 8
			rjmp	L%=minus2			// Relative jump						// 9
			rjmp	L%=end				// Relative jump						// 10
			rjmp	L%=minus1			// Relative jump						// 11
			rjmp	L%=plus2			// Relative jump						// 12
			rjmp	L%=minus1			// Relative jump						// 13
			rjmp	L%=plus1			// Relative jump						// 14
			rjmp	L%=end				// Relative jump						// 15

		//
		L%=minus2:				
			subi	r22, 2				// Substract constant from register		
			sbci	r23, 0				// Substract with carry, constant from register
			sbci	r24, 0				// Substract with carry, constant from register
			sbci	r25, 0				// Substract with carry, constant from register
			rjmp	L%=store			// Relative jump

		//
		L%=minus1:				
			subi	r22, 1				// Substract constant from register
			sbci	r23, 0				// Substract with carry, constant from register
			sbci	r24, 0				// Substract with carry, constant from register
			sbci	r25, 0				// Substract with carry, constant from register
			rjmp	L%=store			// Relative jump

		//
		L%=plus2:				
			subi	r22, 254			// Substract constant from register
			rjmp	L%=z				// Relative jump

		//
		L%=plus1:				
			subi	r22, 255			// Substract constant from register

		//
		L%=z:
			sbci	r23, 255			// Substract with carry, constant from register
			sbci	r24, 255			// Substract with carry, constant from register
			sbci	r25, 255			// Substract with carry, constant from register

		//
		L%=store:				
			st	-X, r25					// Store indirect and pre-dec
			st	-X, r24					// Store indirect and pre-dec
			st	-X, r23					// Store indirect and pre-dec
			st	-X, r22					// Store indirect and pre-dec

		//
		L%=end:

		: : x (arg) : r22, r23, r24, r25, r30, r31