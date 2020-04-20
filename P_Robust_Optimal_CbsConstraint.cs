using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace CPF_experiment
{
    public class P_Robust_Optimal_CbsConstraint : IComparable
    {
        public bool mustConstraint = false;
        public byte agentNum {get; protected set;}
        public byte agentNum2 { get; protected set; }
        public TimedMove move {get; protected set;}
        public TimedMove move2 { get; protected set; }
        public bool queryInstance = false;
        public int constaintRange = 0;

        public P_Robust_Optimal_CbsConstraint(int agentNum, int posX, int posY, Move.Direction direction, int timeStep, int constraintRange = 0, int agentNum2 = -1, int posX2 = -1, int posY2 = -1, Move.Direction direction2 = Move.Direction.NO_DIRECTION, int timeStep2 = -1)
        {
            this.Init(agentNum, posX, posY, direction, timeStep, constraintRange, agentNum2, posX2, posY2, direction2, timeStep2);
        }

        public P_Robust_Optimal_CbsConstraint(int agentNum, TimedMove move, int constraintRange = 0)
        {
            this.Init(agentNum, move, constraintRange);
        }

        public P_Robust_Optimal_CbsConstraint() : this(-1, -1, -1, Move.Direction.NO_DIRECTION, -1) {} // Nonsense values until Init, just allocate move

        public P_Robust_Optimal_CbsConstraint(CbsConflict conflict, ProblemInstance instance, bool agentA, Run.ConstraintPolicy constraintPolicy = Run.ConstraintPolicy.Single, int constraintRange = 0, bool thirdChild = false)
        {
            Move move;
            Move move2;
            int agentNum;
            int agentNum2;
            int minTime;
            this.constaintRange = Math.Abs(conflict.timeStepAgentB - conflict.timeStepAgentA);
            if (constraintPolicy == Run.ConstraintPolicy.Range)
                minTime = Math.Min(conflict.timeStepAgentA, conflict.timeStepAgentB);
            else if (constraintPolicy == Run.ConstraintPolicy.DoubleRange)
                minTime = conflict.timeStepAgentA;
            else
            {
                if (agentA)
                    minTime = conflict.timeStepAgentA;
                else
                    minTime = conflict.timeStepAgentB;
            }
            if (!thirdChild)
            {
                if (agentA)
                {
                    move = conflict.agentAmove;
                    agentNum = instance.m_vAgents[conflict.agentAIndex].agent.agentNum;
                    this.move = new TimedMove(move, minTime);
                }
                else
                {
                    move = conflict.agentBmove;
                    agentNum = instance.m_vAgents[conflict.agentBIndex].agent.agentNum;
                    this.move = new TimedMove(move, minTime);
                }
            }
            else
            {
                mustConstraint = true;
                move = conflict.agentAmove;
                agentNum = instance.m_vAgents[conflict.agentAIndex].agent.agentNum;
                this.move = new TimedMove(move, conflict.timeStepAgentA);
                this.move.direction = Move.Direction.NO_DIRECTION;

                move2 = conflict.agentBmove;
                agentNum2 = instance.m_vAgents[conflict.agentBIndex].agent.agentNum;
                this.move2 = new TimedMove(move2, conflict.timeStepAgentB);
                this.move2.direction = Move.Direction.NO_DIRECTION;

                this.agentNum2 = (byte)agentNum2;
            }
            this.agentNum = (byte)agentNum;
            

            if (conflict.vertex)
                this.move.direction = Move.Direction.NO_DIRECTION;
        }

        public void Init(int agentNum, int posX, int posY, Move.Direction direction, int timeStep, int constraintRange = 0, int agentNum2 = -1, int posX2 = -1, int posY2 = -1, Move.Direction direction2 = Move.Direction.NO_DIRECTION, int timeStep2 = -1)
        {
            if (agentNum2 == -1)
                this.Init(agentNum, new TimedMove(posX, posY, direction, timeStep), constraintRange);
            else
                this.Init(agentNum, new TimedMove(posX, posY, direction, timeStep), constraintRange, agentNum2, new TimedMove(posX2, posY2, direction2, timeStep2));
        }

        public void Init(int agentNum, TimedMove move, int constraintRange = 0, int agentNum2 = -1, TimedMove move2 = null)
        {
            this.agentNum = (byte)agentNum;
            this.move = move;
            this.constaintRange = constraintRange;
            if (agentNum2 != -1)
            {
                mustConstraint = true;
                this.agentNum2 = (byte)agentNum2;
                this.move2 = move2;
            }
        }

        public int time
        {
            get
            {
                return this.move.time;
            }
        }

        /// <summary>
        /// Checks that the agentNum is equal, and compares the move.
        /// If one of the constraints is a query, an instance only created and used to quickly search for a move in a set of constraints,
        /// the direction is ignored if the other constraint is a vertex constraint.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            P_Robust_Optimal_CbsConstraint other = (P_Robust_Optimal_CbsConstraint)obj;
            if (this.agentNum != other.agentNum)
                return false;

            Debug.Assert(this.queryInstance == false || other.queryInstance == false); // At most one of the instances is a query
            Debug.Assert(this.queryInstance == false || this.move.direction != Move.Direction.NO_DIRECTION); // Must query regarding a specific direction
            Debug.Assert(other.queryInstance == false || other.move.direction != Move.Direction.NO_DIRECTION); // Must query regarding a specific direction
            if (this.queryInstance || other.queryInstance) // This way if the constraint is a vertex constraint than it will be equal to a query containing a move from any direction to that position,
                                                           // and if it is an edge constraint than it will only be equal to queries containing a move from that specific direction to that position.
                return this.move.Equals(other.move);
            else // A vertex constraint is different to an edge constraint for the same agentNum and position.
                 // Must check the direction explicitly because vertex constraints have no direction and moves with no direction
                 // compare equal to moves with any direction
                return this.move.Equals(other.move) && this.move.direction == other.move.direction; 
        }

        /// <summary>
        /// Uses the move and the agents.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                ans += this.move.GetHashCode() * 3;
                ans += this.agentNum * 5;
                if (this.constaintRange != -1)
                    ans += this.constaintRange * 11;
                if (this.agentNum2 != -1)
                    ans += this.agentNum2 * 11;
                return ans;
            }
        }

        public int GetTimeStep() { return this.move.time; } // FIXME: Make this into a property

        public Move.Direction GetDirection()
        {
            return this.move.direction;
        }
        
        public override string ToString()
        {
            return move.ToString() + "-" + move.direction.ToString().PadRight(12) + " time=" + move.time + " agentNum " + agentNum + "";
        }

        /// <summary>
        /// Kind of the opposite of Equals: checks that the moves are unequal or that not one of the other's agents appears in this.agents.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool Allows(P_Robust_Optimal_CbsConstraint other)
        {
            if (this.move.Equals(other.move) == false) // Minor behavior change: if exactly one move has a set direction, and they're otherwise equal the method used to return true.
                return true;
            if (this.agentNum == other.agentNum)
                return false;
            return true;
        }

        public int CompareTo(object item)
        {
            P_Robust_Optimal_CbsConstraint other = (P_Robust_Optimal_CbsConstraint)item;

            return this.move.time.CompareTo(other.move.time);
        }

        public bool ViolatesMustConstraint(byte agent, TimedMove move)
        {
            if (this.agentNum != agent)
                return false;
            return this.move.Equals(move) == false;
        }
    }
}
