//emulator updated
#include "emulib.h"
#include "math.h"
typedef struct {
    int32_t reg[16];
    int32_t cpsr; 
}tCPU;

#define max_rom 0x1FFFFFFF
#define min_rom 0x00000000

#define max_ram 0x3FFFFFFF
#define min_ram 0x20000000

#define max_per 0x5FFFFFFF
#define min_per 0x40000000

uint8_t rom[0x200000];
uint8_t ram[0x100000];
tCPU cpu;

int N_flag;
int Z_flag;	
int C_flag_add;
int C_flag_sub;
int C_flag_shift;
int V_flag_add;
int V_flag_sub;
void memory_write(uint32_t mem_val, uint32_t mem_addr);
void memory_read(uint32_t *target, uint32_t mem_addr);
void update_N_flag(uint32_t rc,uint32_t cpsr);
void update_Z_flag(uint32_t rc,uint32_t cpsr);
void update_C_flag_add(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr);
void update_V_flag_add(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr);
void update_C_flag_sub(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr);
void update_V_flag_sub(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr);
void update_C_flag_shift (uint32_t rb,uint32_t cpsr);
		
		
//Fetches an instruction from the given address
//and returns its encoded value
int16_t fetch(uint32_t addr)
{
	//to be implemented
	return 0;
}

void memory_write(uint32_t mem_val, uint32_t mem_addr) {

    mem_addr &= ~(0b11);

    if(mem_addr >= min_per && mem_addr <= max_per)
	{	
        peripheral_write(mem_addr, mem_val);
	}	
    else if(mem_addr >= min_rom && mem_addr <= max_rom)
	{
        *(uint32_t*)(rom + mem_addr - min_rom) = mem_val;
    }
    else if(mem_addr >= min_ram && mem_addr <= max_ram) 
	{
        *(uint32_t*)(ram + mem_addr - min_ram) = mem_val;
    }


}

void memory_read(uint32_t *target, uint32_t mem_addr) {

    mem_addr &= ~(0b11);
    
    int mem_t;

    if(mem_addr >= min_per && mem_addr <= max_per)
	{	
        peripheral_read(mem_addr, target);
	}
	else if(mem_addr >= min_rom && mem_addr <= max_rom) {
        mem_t = mem_addr - min_rom;
        *target = *(uint32_t*)(&rom[mem_t]);
    }
    else if(mem_addr >= min_ram && mem_addr <= max_ram) { 
        mem_t = mem_addr - min_ram;   
        *target = *(uint32_t*)(&ram[mem_t]);
    }
    
}
//Fetches an instruction from ROM, decodes and executes it

void update_N_flag(uint32_t rc,uint32_t cpsr){
	
	N_flag = (rc >> 31) & 0x1;
	cpsr = (N_flag == 1) ? cpsr | (1u<<31) : cpsr & ~(1u<<31) ; //negative
}

void update_Z_flag(uint32_t rc,uint32_t cpsr){
	
	Z_flag = (rc == 0 ) ? 0x1 : 0x0;
	cpu.cpsr = (Z_flag == 1) ? cpu.cpsr | (1u<<30) : cpu.cpsr & ~(1u<<30) ; //zero
}

void update_C_flag_add(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr){
	
	C_flag_add = ((((ra>>31)&0x1) & ((rb>>31)&0x1)) | (((ra>>31)&0x1) & (~(rc>>31)&0x1)) | (((rb>>31)&0x1) & (~(rc>>31)&0x1))); 
	//return ((isNegative(op1Value) && isNegative(op2Value)) || (isNegative(op1Value) && isPositive(result)) || (isNegative(op2Value) && isPositive(result)));
	cpsr = (C_flag_add == 1) ? cpsr | (1u<<29) : cpsr & ~(1u<<29) ; //carry
}

void update_V_flag_add(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr){
		
	V_flag_add =  ((((ra>>31)&0x1) & ((rb>>31)&0x1)) & (~(rc>>31)&0x1) | ((~(ra>>31)&0x1)) &(~(rb>>31)&0x1) & ((rc>>31)&0x1)); 
	// return ((isNegative(op1Value) && isNegative(op2Value) && isPositive(result)) || (isPositive(op1Value) && isPositive(op2Value) && isNegative(result)));
	cpsr = (V_flag_add == 1) ? cpsr | (1u<<28) : cpsr & ~(1u<<28) ; //overflow
		
}


void update_C_flag_sub(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr){
		
	C_flag_sub = ((((ra>>31)&0x1) & (~(rb>>31)&0x1)) | (((ra>>31)&0x1) & (~(rc>>31)&0x1)) | ((~(rb>>31)&0x1) & (~(rc>>31)&0x1))); 
	//return ((isNegative(op1Value) && isPositive(op2Value)) || (isNegative(op1Value) && isPositive(result)) || (isPositive(op2Value) && isPositive(result)));
	cpsr = (C_flag_sub == 1) ? cpsr | (1u<<29) : cpsr & ~(1u<<29) ; //carry
}	
void update_V_flag_sub(uint32_t ra,uint32_t rb,uint32_t rc,uint32_t cpsr){
	
	V_flag_sub =  (((((ra>>31)&0x1) & (~(rb>>31)&0x1)) & (~(rc>>31)&0x1)) | (((~(ra>>31)&0x1)) &((rb>>31)&0x1) & ((rc>>31)&0x1))); 
	//return ((isNegative(op1Value) && isPositive(op2Value) && isPositive(result)) || (isPositive(op1Value) && isNegative(op2Value) && isNegative(result)));
	cpu.cpsr = (V_flag_sub == 1) ? cpu.cpsr | (1u<<28) : cpu.cpsr & ~(1u<<28) ; //overflow
		
}

void update_C_flag_shift (uint32_t rb,uint32_t cpsr){
	
		C_flag_shift = (rb!=0x0) ? 0x1 : 0x0;
		cpsr = (C_flag_shift == 1) ? cpsr | (1u<<29) : cpsr & ~(1u<<29) ; //carry
}	
int32_t execute(void)
{
	uint32_t pc;
	uint32_t sp;
	uint32_t inst;
	uint32_t ra, rb, rc;
	uint32_t rm, rd, rn, rs;
	uint32_t op;
	uint16_t X;

	uint32_t temp;
	
	
	pc = cpu.reg[15];

	X = pc - 2;
	inst = rom[X] | rom[X+1] << 8;
	pc += 2;
	cpu.reg[15] = pc;
	
	//ORR
	if ((inst & 0xFFC0) == 0x4300)
	{
		rd = (inst >> 0) & 0x7;
		rm = (inst >> 3) & 0x7;
		////fprintf(stderr, "orrs r%u,r%u\n", rd, rm);
		ra = cpu.reg[rd];
		rb = cpu.reg[rm];
		rc = ra | rb;
		cpu.reg[rd] = rc;
		
		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		return(0);
	}
		//AND
	else if ((inst & 0xFFC0) == 0x4000)
	{
		rd = (inst >> 0) & 0x7;
		rm = (inst >> 3) & 0x7;
		////fprintf(stderr, "ands r%u,r%u\n", rd, rm);
		ra = cpu.reg[rd];
		rb = cpu.reg[rm];
		rc = ra & rb;
		cpu.reg[rd] = rc;
		
		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		
		return(0);
	}
	//MOV #imm8
	else if ((inst & 0xF800) == 0x2000)
	{
		rm = (inst >> 0) & 0xff;
		rd = (inst >> 8) & 0x7;
		//fprintf(stderr, "movs #imm8 r%u,r%u\n", rd, rm);
		//fprintf(stderr,"sonra cpu.cpsr=%x rm=%x pc=%x inst=%x cpu.reg[rd]=%x\n",cpu.cpsr,rm,pc,inst,cpu.reg[rd]);

		cpu.reg[rd]=rm;

		update_N_flag (rm, cpu.cpsr);		
		update_Z_flag (rm, cpu.cpsr);
		//fprintf(stderr,"sonra cpu.cpsr=%x rm=%x pc=%x inst=%x cpu.reg[rd]=%x\n",cpu.cpsr,rm,pc,inst,cpu.reg[rd]);
		
		//fprintf(stderr,"rm=%x rd=%x  ", rm, rd);

		return(0);
	}
	// MOV Hd, Lm
		
	else if ((inst & 0xFFC0) == 0x4680)
	{
		rd = ((inst >> 3) & 0x7); 	 //low register value
		rm = ((inst >> 0) & 0x7)|0x8; //high register value
		//fprintf(stderr, "MOV  Hd, Lm r%u,r%u\n", rd, rm);		
		cpu.reg[rm]=cpu.reg[rd];
		
		return(0);
	}


		// MOV Ld, Hm
		else if ((inst & 0xFFC0) == 0x4640)
	{
		rd = ((inst >> 0) & 0x7); 	  //low register value
		rm = ((inst >> 3) & 0x7)|0x8; //high register value
		//fprintf(stderr, "MOV  Ld, Hm r%u,r%u\n", rd, rm);		
		cpu.reg[rd]=cpu.reg[rm];
		
		return(0);
	}

		//  MOV Hd, Hm
		else if ((inst & 0xFFC0) == 0x46C0)
	{
		rd = ((inst >> 0) & 0x7)|0x8; //low register value
		rm = ((inst >> 3)& 0x7)|0x8; //high register value
		//fprintf(stderr, "MOV  Hd, Hm r%u,r%u\n", rd, rm);		
		cpu.reg[rd]=cpu.reg[rm];
		
		return(0);
	}	
	
		//ADD all register lo
	else if ((inst & 0xFE00) == 0x1800)
	{
		rd = (inst >> 0) & 0x7;
		rn = (inst >> 3) & 0x7;
		rm = (inst >> 6) & 0x7;
		//fprintf(stderr, "ADD  1 r%u,r%u\n", rd, rm);
		ra = cpu.reg[rn];
		rb = cpu.reg[rm];
		rc = (ra + rb);
		cpu.reg[rd] = rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_add (ra,rb,rc,cpu.cpsr);
		update_V_flag_add (ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
	
	
		//ADD immediate 3
	else if ((inst & 0xFE00) == 0x1C00)
	{
		rd = (inst >> 0) & 0x7;
		rn = (inst >> 3) & 0x7;
		rb = (inst >> 6) & 0x7;
		//fprintf(stderr, "ADD immediate 3 r%u,r%u\n", rd, rm);
		ra = cpu.reg[rn];
	//	rb = cpu.reg[rm];
		rc = (ra + rb);
		cpu.reg[rd] = rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_add (ra,rb,rc,cpu.cpsr);
		update_V_flag_add (ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
	
			//ADD immediate 8
	else if ((inst & 0xF800) == 0x3000)
	{
		rd = (inst >> 8) & 0x7;
		rb = (inst >> 0) & 0xff; //immed
		//fprintf(stderr, "ADD immediate 8 r%u,r%u\n", rd, rb);
		uint32_t ra = cpu.reg[rd];
		//fprintf(stderr,"inst=%x cpu.cpsr=%x rd=%x cpu.reg[rd]=%x rb=%x cpu.reg[rc]=%x\n",inst, cpu.cpsr, rd,cpu.reg[rd],rb, cpu.reg[rc] );

		uint32_t rc = (ra + rb);
		cpu.reg[rd] =  rc;
		
		
		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_add (ra,rb,rc,cpu.cpsr);
		update_V_flag_add (ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
	
				//ADD ld hm
	else if ((inst & 0xFFC0) == 0x4440)
	{
		rd = (inst >> 0) & 0x7;
		rm = ((inst >> 3) & 0x7)|0x8;  // |0x8i�lemi ??
		//fprintf(stderr, "ADD ld, hm r%u,r%u\n", rd, rm);
		ra = cpu.reg[rd];
		rb = cpu.reg[rm];
		rc = (ra + rb);
		cpu.reg[rd] = rc;
	
		return(0);
	}
	
		//ADD Hd LM
		else if ((inst & 0xFFC0) == 0x4480)
	{
		rd = ((inst >> 0) & 0x7)|0x8;
		rm = ((inst >> 3) & 0x7);
		//fprintf(stderr, "ADD Hd LM r%u,r%u\n", rd, rm);
		ra = cpu.reg[rd];
		rb = cpu.reg[rm];
		rc = (ra + rb);
		cpu.reg[rd] = rc;
		
		return(0);
	}

		//ADD Hd Hm
		else if ((inst & 0xFFC0) == 0x44C0)
	{
		rd = ((inst >> 0) & 0x7)|0x8;
		rm = ((inst >> 3) & 0x7)|0x8;
		//fprintf(stderr, "ADD Hd Hm 8 r%u,r%u\n", rd, rm);
		ra = cpu.reg[rd];
		rb = cpu.reg[rm];
		rc = (ra + rb);
		cpu.reg[rd] =  rc;
		
		return(0);
	}

			//ADD Ld, pc, #immed*4 
		else if ((inst & 0xF800) == 0xA000)
	{
		rd = ((inst >> 8) & 0x7);
		rm = ((inst >> 0) & 0xff);
		//fprintf(stderr, "ADD Ld, pc, #immed*4 r%u,r%u\n", rd, rm);
		ra = cpu.reg[15]; //pc value
		rc = (ra + rm*4);
		cpu.reg[rd] = rc;
		
		return(0);
	}

				//ADD Ld, sp, #immed*4 
		else if ((inst & 0xF800) == 0xA800)
	{
		rd = ((inst >> 8) & 0x7);
		rm = ((inst >> 0) & 0xff);
		//fprintf(stderr, "ADD Ld, sp, #immed*4 ,r%u,r%u\n", rd, rm);
		ra = cpu.reg[13]; //sp value
		rc = (ra + rm*4);
		cpu.reg[rd] = rc;
		
		return(0);
	}
	
					//ADD sp, #immed*4
		else if ((inst & 0xFF80) == 0xB000)
	{
		//rd = ((inst >> 8) & 0x7);
		rm = ((inst >> 0) & 0x7F);
		//fprintf(stderr, "ADD sp , #immed*4 ,r%u,r%u\n", rd, rm);
		ra = cpu.reg[13]; //sp value
		rc = (ra + rm*4);
		cpu.reg[13] = rc;
		
		return(0);
	}

			//SUB
	else if ((inst & 0xFE00) == 0x1A00)
	{
		rd = (inst >> 0) & 0x7;
		rn = (inst >> 3) & 0x7;
		rm = (inst >> 6) & 0x7;
		//fprintf(stderr, "SUB r%u,r%u\n", rd, rm);
		ra = cpu.reg[rn];
		rb = cpu.reg[rm];
		rc = (ra - rb);
		cpu.reg[rd] = rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}

			//SUB immediate 3
	else if ((inst & 0xFE00) == 0x1E00)
	{
		rd = (inst >> 0) & 0x7;
		rn = (inst >> 3) & 0x7;
		rb = (inst >> 6) & 0x7;
		//fprintf(stderr, "SUB immediate 3 r%u,r%u\n", rd, rm);
		ra = cpu.reg[rn];
		rc =  (ra - rb);
		cpu.reg[rd] =  rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}

			//SUB immediate 8
	else if ((inst & 0xF800) == 0x3800)
	{
		rd = (inst >> 8) & 0x7;
		rb = (inst >> 0) & 0xff;
		//fprintf(stderr, "SUB immediate 8 r%u,r%u\n", rd, rm);
		ra = cpu.reg[rd];
		rc = (ra - rb);
		cpu.reg[rd] = rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
					//SUB sp, #immed*4
		else if ((inst & 0xFF80) == 0xB080)
	{
		//rd = ((inst >> 8) & 0x7);
		rm = ((inst >> 0) & 0x7F);
		//fprintf(stderr, "SUB sp, #immed*4 ,r%u,r%u\n", rd, rm);
		ra = cpu.reg[13]; //sp value
		rc = (ra - rm*4);
		cpu.reg[13] =  rc;
		
		return(0);
	}
		//LDR Ld, [pc, #immed*4]
	else if ((inst & 0xF800) == 0x4800)
	{
		rd = (inst >> 8) & 0x7;  //Ld
		rn = (inst >> 0) & 0xFF; //imm8
		//fprintf(stderr, "LDR Ld, [pc, #immed*4] r%u,r%u\n", rd, rn);
		
		ra = pc + rn * 4;
		//fprintf(stderr,"once ra=%x pc=%x inst=%x rn=%x\n",ra,pc,inst,rn);
		
        memory_read( &cpu.reg[rd], pc + rn * 4);
		//fprintf(stderr,"sonra ra=%x pc=%x inst=%x cpu.reg[rn]=%x rd=%x, cpu.reg[rd]=%x\n",ra,pc,inst,cpu.reg[rn],rd,cpu.reg[rd]);
		
		return(0);
	}

			//LDR 
	else if ((inst & 0xFE00) == 0x5800)
	{	rd = ((inst >> 0) & 0x7);
		rm = ((inst >> 6) & 0x7);
		rn = (inst >> 3) & 0x7;

		//fprintf(stderr, "LDR  r%u,r%u\n", rd, rn);
		
        uint32_t ra = cpu.reg[rm] + cpu.reg[rn];

        memory_read( &cpu.reg[rd], ra);			
		/*cpu.reg[rd]=ra+rb;*/
		
		return(0);
	}
	
		//LDR Ld, [sp, #immed*4
	else if ((inst & 0xF800) == 0x9800){
		rd = (inst >> 8) & 0x7;
		rn = (inst >> 0) & 0xFF;
		
		//fprintf(stderr, "LDR Ld, [sp, #immed*4 r%u,r%u\n", rd, rn);
        
		uint32_t ra = cpu.reg[13] + rn * 4;

        memory_read( &cpu.reg[rd], ra);
		/*cpu.reg[rd]=(rn*4)+sp;*/
		
		return(0);
	}

				// LDR Ld, [Ln, #immed*4] 
	else if ((inst & 0xFE00) == 0x5800)
	{	rd = ((inst >> 0) & 0x7);
		rm = ((inst >> 6) & 0x1F); //immed
		rn = (inst >> 3) & 0x7;
		//fprintf(stderr, "DR Ld, [Ln, #immed*4]   r%u,r%u\n", rd, rn);
		//fprintf(stderr,"�nce cpu.cpsr=%x rm=%x pc=%x inst=%x cpu.reg[rn]=%x cpu.reg[rd]=%x\n",cpu.cpsr,rm,pc,inst, cpu.reg[rn], cpu.reg[rd]);

		uint32_t ra = cpu.reg[rn] + 4 * rm;

		memory_read( &cpu.reg[rd], ra);
		//fprintf(stderr,"sonra cpu.cpsr=%x rm=%x pc=%x inst=%x cpu.reg[rn]=%x cpu.reg[rd]=%x\n",cpu.cpsr,rm,pc,inst, cpu.reg[rn], cpu.reg[rd]);

		return(0);
	}

		
		// CMP Ld, #immed8	
		else if ((inst & 0xF800) == 0x2800)
	{
		rb = (inst >> 0) & 0xff;//immed8
		rn = (inst >> 8) & 0x7 ;//LOW REG RM
		//fprintf(stderr, "CMP Ld, #imm8 r%u,%u\n", rn, rb);
		ra = cpu.reg[rn];
		rc = (ra - rb); 
		//fprintf(stderr, "cpu.cpsr=%x,ra=%x,rb=%x\n",cpu.cpsr, ra, rb);
		
		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
			// 	CMP Hm, Ld
		else if ((inst & 0xFFC0) == 0x4540)
	{
		rn = (inst >> 0) & 0x7;//LOW REG
		rm = (inst >> 3) & 0x7 | 0x8;//HIGH REG
		//fprintf(stderr, "CMP r%u,r%u\n", rn, rm);
		ra = cpu.reg[rn];
		rb = cpu.reg[rm];
		rc = (ra - rb); //33. bit nedenn??

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
	
		// 	CMP  Hn ,Lm
		else if ((inst & 0xFFC0) == 0x4580)
	{
		rn = (inst >> 3) & 0x7;//LOW REG
		rm = (inst >> 0) & 0x7 | 0x8;//HIGH REG
		//fprintf(stderr, "CMP Hn ,Lm r%u,r%u\n", rn, rm);
		ra = cpu.reg[rn];
		rb = cpu.reg[rm];
		rc = (ra - rb); 

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
		// 	CMP  Hn ,Hm
		else if ((inst & 0xFFC0) == 0x45C0)
	{
		rn = (inst >> 0) & 0x7;//LOW REG RD
		rm = (inst >> 3) & 0x7 | 0x8 ;//HIGH REG RM
		//fprintf(stderr, "CMP Hn ,Hm r%u,r%u\n", rn, rm);
		ra = cpu.reg[rn];
		rb = cpu.reg[rm];
		rc = (ra - rb); 

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
	
		// CMP Ld, lm	
		else if ((inst & 0xFFC0) == 0x4280)
	{
		rn = (inst >> 0) & 0x7;//LOW REG Rn/Rd
		rm = (inst >> 3) & 0x7;//LOW REG Rm
		//fprintf(stderr, "CMP Ln , Lm r%u,r%u\n", rn, rm);
		ra = cpu.reg[rn];
		rb = cpu.reg[rm];		
		rc = (ra - rb);

		//fprintf(stderr,"inst=%x cpu.cpsr=%x ra=%x  rb=%x rc=%x\n",inst, cpu.cpsr, ra,rb,rc );
		
		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_sub(ra,rb,rc,cpu.cpsr);
		update_V_flag_sub(ra,rb,rc,cpu.cpsr);
		
		return(0);
	}
    				//LSLS Rd, Rm #<shift> 
		else if ((inst & 0xF800) == 0x0000)
	{
		rd = ((inst >> 0) & 0x7);   //Ld
		rm = ((inst >> 3) & 0x7);	//Lm
		rn = ((inst >> 6) & 0xf1); 	//immed5
		//fprintf(stderr, "LSLS Rd, Rm, #<shift> ,r%u,r%u,r%u\n", rd, rm, rn);
		ra = cpu.reg[rm];
		rc = ra*((int32_t)pow(2,rb));
		cpu.reg[rd] =  rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_shift (rb, cpu.cpsr);
		
		return(0);
	}
				//LSLS Rd, Rs
		else if ((inst & 0xFFC0) == 0x4080)
	{
		rd = ((inst >> 0) & 0x7);   //Ld
		rs = ((inst >> 3) & 0x7);	//Lm
		//fprintf(stderr, "LSLS Rd, Rs, r%u,r%u,r%u\n", rd, rs );
		ra = cpu.reg[rd];
		rb = cpu.reg[rs];
        rc=	ra*((int32_t)pow(2,rb));
		cpu.reg[rd] =  rc;
		//fprintf(stderr, "cpu.cpsr=%x ra=%x, rb=%x, rc=%x \n", cpu.cpsr, ra, rb, rc);

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_shift (rb, cpu.cpsr);

		return(0);
	}
    				//LSRS Rd, Rs , #<shift>
		else if ((inst & 0xF800) == 0x0800)
	{
		rd = ((inst >> 0) & 0x7);   //Ld
		rm = ((inst >> 3) & 0x7);	//Lm
		rn = ((inst >> 6) & 0xf1); 	//immed5 
		//fprintf(stderr, "LSRS Rd, Rs, #<shift>, r%u,r%u,r%u\n", rd, rm, rn );
		ra = cpu.reg[rm];
		rb = cpu.reg[rn];		
		rc = ra/((int32_t)pow(2,rb+1));
        cpu.reg[rd] =  rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		C_flag_shift = 0x1;		 
		cpu.cpsr = cpu.cpsr | (1u<<29)  ; //carry
		
		return(0);
	}
					//LSRS Rd, Rs
		else if ((inst & 0xFFC0) == 0x40C0)
	{
		rd = ((inst >> 0) & 0x7);   //Ld
		rs = ((inst >> 3) & 0x7);	//Lm
		//fprintf(stderr, "LSLS Rd, Rs, r%u,r%u \n", rd, rm );
		ra = cpu.reg[rm];
		rb = cpu.reg[rs];
        rc = ra/((int32_t)pow(2,rb));
		cpu.reg[rd] =  rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_shift (rb, cpu.cpsr);

		return(0);
	}

    				//ASR, #<shift>
		else if ((inst & 0xF800) == 0x1000)
	{
		rd = ((inst >> 0) & 0x7);   //Ld
		rm = ((inst >> 3) & 0x7);	//Lm
		rn = ((inst >> 6) & 0xf1); 	//immed5 
		//fprintf(stderr, "ASR #<shift>, r%u,r%u,r%u\n", rd, rm, rn );
		ra = cpu.reg[rm];
	
		rc= (ra/(int32_t)pow(2,rn+1))|((ra & (int32_t)pow(2,rn+1)-1)<<(32-rn-1));
        cpu.reg[rd]=rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		C_flag_shift = 0x1;		 
		cpu.cpsr = cpu.cpsr | (1u<<29)  ; //carry
		
		return(0);
	}


    		//ASR VDRS
		else if ((inst & 0xFFC0) == 0x4100)
	{
		rd = ((inst >> 0) & 0x7);   //Ld
		rs = ((inst >> 3) & 0x7);	//Lm
		//fprintf(stderr, "ASR VDRS r%u,r%u \n", rd, rs );
		ra = cpu.reg[rd];
		rb = cpu.reg[rs];	
		rc= (ra/(int32_t)pow(2,rb))|((ra & (int32_t)pow(2,rb)-1)<<(32-rb));
        cpu.reg[rd]=rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		update_C_flag_shift (rb, cpu.cpsr);

		return(0);
	}

    		//  STR Ld, Lm, Ln
		else if ((inst & 0xFE00) == 0x5000)
	{
		rd = ((inst >> 0) & 0x7); //low register directory value
		rn = ((inst >> 3) & 0x7); //low register load value
		rm = ((inst >> 6) & 0x7); //low register load value	
		//fprintf(stderr, "ilk STR Ld, Ln, Lm r%u,r%u,r%u\n", rd, rn, rm);
		
		uint32_t ra = cpu.reg[rm] + cpu.reg[rn];

        memory_write( cpu.reg[rd], ra);
 
		return(0);
	}

            	//  STR  [Ln, #immed5
		else if ((inst & 0xF800) == 0x6000)
	{
		rd = ((inst >> 0) & 0x7); //low register directory value ld
		rn = ((inst >> 3)& 0x7); //low register load value ln
		uint8_t immed = ((inst >> 6)& 0x1F);//low register load value	immed5
		//fprintf(stderr, "STR  [Ln, #imm5 r%u,r%u,r%u\n", rd, rn, rm);
		//ra = cpu.reg[rn]+rm*4;
		//fprintf(stderr,"inst=%x cpu.cpsr=%x rd=%x cpu.reg[rn]=%x rm=%x ra=%x cpu.reg[rd]=%x\n",inst, cpu.cpsr, rd,cpu.reg[rn],rm, ra, cpu.reg[rd] );
        uint32_t ra = cpu.reg[rn] + 4 * immed;

        memory_write( cpu.reg[rd], ra);		
		////fprintf(stderr,"inst=%x cpu.cpsr=%x rd=%x cpu.reg[rn]=%x rm=%x ra=%x cpu.reg[rd]=%x\n",inst, cpu.cpsr, rd,cpu.reg[rn],rm, ra, cpu.reg[rd] );
	
		return(0);
        }

            //  STR [sp, #immed*4]
		else if ((inst & 0xF800) == 0x9000)
	{
		rn = ((inst >> 0) & 0xFF);  //imm8
		rd = ((inst >> 8)& 0x7);    
		////fprintf(stderr, "STR [sp, #immed*4] r%u,r%u\n", rd, rn);
		
        uint32_t ra = cpu.reg[13] + rn * 4;

        memory_write( cpu.reg[rd], ra);		
 
		return(0);
        }

            //  PUSH [R,reg_list]
		else if ((inst & 0xFE00) == 0xB400)
	{
		rn = ((inst >> 0) & 0xFF); //reglist
		ra  = ((inst >> 8) & 0x1); //R 
		////fprintf(stderr, "PUSH [R,reg_list] ra%u,rn%u\n", ra, rn);
		rd= cpu.reg[13];
		
		if(ra == 1){
            rd -= 4; 
            memory_write(cpu.reg[14], rd);
        }

        for(int i = 7; i >= 0; i--){
            if((rn & (1 << i)) != 0){
                rd -= 4;
                memory_write( cpu.reg[i], rd);
            }
        }  
        cpu.reg[13] = rd;		
		
		return(0);
        }
		
			// POP [R,reg_list]
    else if ((inst & 0xFF00) == 0xB400)
    {
        rn= (inst>>0) & 0xff; //reglist
		ra = (inst>>8) & 0x1; //R
        rd = cpu.reg[13];
		////fprintf(stderr, "POP [R,reg_list] ra%u,rn%u\n", ra, rn);

        for(int i = 0; i < 8; i++){
            if((rn & (1 << i)) != 0){
                memory_read( &cpu.reg[i], rd);
                rd += 4;
            }
        }
        if(ra == 1){
            memory_read( &cpu.reg[15], rd);
            rd += 4;
        }
        cpu.reg[13] = rd;
		
        return 0;
    }		
		
	           //  BRANCH []
		else if ((inst & 0xF800) == 0xE000)
	{
		temp  = ((inst >> 0) & 0x7ff);
		temp = (((temp>>10) & 0x1) == 1) ? 0xfffff800 | temp : temp;
		////fprintf(stderr, "Branch[] r%d  \n", temp);		
		int32_t temp_rb = cpu.reg[15]+2+temp*2;
		cpu.reg[15] = temp_rb & 0xffff; 
		////fprintf(stderr,"binary = %x \n", ((inst >> 0) & 0x7ff) );		
					
		return(0);
    }	
	
	           //  BRANCH [cond]
		else if (((inst & 0xF000) == 0xD000) & ((inst>>8 & 0xf) < 0b1110 ))
	{			
		uint32_t rn = ((inst >> 0) & 0xff);
		temp = inst & 0xff;
		temp = (((temp>>7) & 0x1) == 1) ? 0xffffff00 | temp : temp;
		uint32_t cond = ((inst >> 8) & 0xf);
		////fprintf(stderr,"cpu.cpsr=%x inst=%x cond=%x\n",cpu.cpsr, inst, cond);
		uint32_t do_branch = 0;
	
		switch ( cond ) {
            case 0:
                if (((cpu.cpsr>>30)&0x1) == 1) {
					////fprintf(stderr,"cdadsapu.cpsr=%x inst=%x cond=%x\n",cpu.cpsr, inst, cond);
					do_branch = 1;
                }
				break;
            case 1:    
				////fprintf(stderr,"cpu.cpsr=%x cpu.cpsr>> =%x inst=%x cond=%x\n",cpu.cpsr, (cpu.cpsr>>30) &0x1 , inst, cond);	
			    if (((cpu.cpsr>>30)&0x1) == 0) {
					////fprintf(stderr,"cpus.cpsr=%x inst=%x cond=%x\n",cpu.cpsr, inst, cond);
					do_branch = 1;
				}			
                break;
            case 2:
                if (((cpu.cpsr>>29)&0x1) == 1)
				do_branch = 1; 
                break;
            case 3:
                if (((cpu.cpsr>>29)&0x1) == 0)
				do_branch = 1;
               break;
            case 4:
                if (((cpu.cpsr>>31)&0x1) == 1) do_branch = 1;
                break;
            case 5:
                if (((cpu.cpsr>>31)&0x1) == 0) do_branch = 1;
                break;
            case 6:
                if (((cpu.cpsr>>28)&0x1) == 1) do_branch = 1;
                break;
            case 7:
                if (((cpu.cpsr>>28)&0x1) == 0) do_branch = 1;
                break;
            case 8:  
                if ((((cpu.cpsr>>29)&0x1) == 1) && (((cpu.cpsr>>30)&0x1) == 0)) do_branch = 1;
                break;
            case 9:
                if ((((cpu.cpsr>>29)&0x1) == 0 || (((cpu.cpsr>>30)&0x1))) == 1) do_branch = 1;
                break;
            case 10:
                if (((cpu.cpsr>>31)&0x1) == ((cpu.cpsr>>28)&0x1 )) do_branch = 1;
                break;
			case 11:
                if (((cpu.cpsr>>31)&0x1)  != ((cpu.cpsr>>28)&0x1)) do_branch = 1;
                break;
            case 12:
                if ((((cpu.cpsr>>30)&0x1) == 0) && (((cpu.cpsr>>31)&0x1) == ((cpu.cpsr>>30)&0x1))) do_branch = 1;
                break;
            case 13:
                if ((((cpu.cpsr>>30)&0x1) == 1) || (((cpu.cpsr>>31)&0x1) != ((cpu.cpsr>>28)&0x1))) do_branch = 1;
                break;
			case 14:
				do_branch=0;
				break;
			case 15:  
				do_branch=0;
				break;
			default: do_branch=0;
				break;			
		}
		////fprintf(stderr,"do_branch=%x\n", do_branch);		
		if( do_branch==1){
		////fprintf(stderr, "Branch [cond] r%u,r%u\n", rn, ra);
		int32_t temp_rb = cpu.reg[15]+2+temp*2;
		cpu.reg[15] = temp_rb & 0xffff ;
		////fprintf(stderr,"binary = %x \n", ((temp >> 0) & 0xff) );		
		}
		else{

		}
	return(0);	
	}
	
	 
	           //  BL
		else if ((inst & 0xF800) == 0xF800)
	{
		//prefix icin
		if((inst & 0xF800 )== 0xF000)
		{
		rm = (inst >> 0) & 0x7FF; //poff
		rn = (inst >> 0) & 0x7ff; 
		////fprintf(stderr, "BL  r%u\n", rn );
		rb = inst+4+(rm<<12)+rn*2;
    	cpu.reg[14] = rom[pc - 2] | rom[pc -1] << 8;
		cpu.reg[15] = rb; 
		
		return(0);
		}			
    }
	
	           //  BX
		else if ((inst & 0xFFC0) == 0x4700)
	{
		rm = ((inst >> 3) & 0x7);
		////fprintf(stderr, "BX  r%u\n", rm );
		cpu.reg[15] = rm & 0xFFFFFFFF; 
		if (rm& 0x1 == 0x0 ){
		cpu.cpsr |= 0x100000 ;
		}
		
		return(0);			
    }

		//MUL
	else if ((inst & 0xFFC0) == 0x4340)
	{
		rd = (inst >> 0) & 0xff;
		rm = (inst >> 3) & 0x7;
		////fprintf(stderr, "mul r%u,r%u\n", rd, rm);
		ra = cpu.reg[rd];
		rb = cpu.reg[rm];
		rc = 0xffffffff & (ra * rb);
		cpu.reg[rd] = rc;

		update_N_flag (rc, cpu.cpsr);		
		update_Z_flag (rc, cpu.cpsr);
		
		return(0);
	}

	//Rest of the instructions to be implemented here

	////fprintf(stderr, "invalid instruction 0x%08X 0x%04X\n", pc - 4, inst);
	return(1);
}

//Resets the CPU and initializes the registers
int32_t reset(void)
{
	
	cpu.cpsr = 0;

	cpu.reg[14] = 0xFFFFFFFF;
	//First 4 bytes in ROM specifies initializes the stack pointer
	cpu.reg[13] = rom[0] | rom[1] << 8 | rom[2] << 16 | rom[3] << 24;
	//Following 4 bytes in ROM initializes the PC
	cpu.reg[15] = rom[4] | rom[5] << 8 | rom[6] << 16 | rom[7] << 24;
	cpu.reg[15] += 2;	

    uint32_t pc;
    uint32_t sp;
	pc= cpu.reg[15];
	sp= cpu.reg[13];
	
	return(0);
}

//Emulator loop
int32_t run(void)
{
	reset();
	while (1)
	{
		if (execute()) break;
	}
	return(0);
}

//Emulator main function
int32_t main(int32_t argc, char* argv[])
{
	if (argc < 2)
	{
		////fprintf(stderr, "input assembly file not specified\n");
		return(1);
	}

	memset(rom, 0xFF, sizeof(rom));
	
	system_init();

	if (load_program(argv[1], rom) < 0)
	{
		return(1);
	}
		
	memset(ram, 0x00, sizeof(ram));
	
	run();
	system_deinit();

	return(0);
}
