using System;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    /// <summary>
    /// Describes a Move at a given timestep.
    /// </summary>
    public class TimedMove  : Move
    {
        public int time;

//        public static const int FOREVER_AFTER = int.MaxValue

        public TimedMove(int x, int y, Move.Direction direction, int time)
            : base(x, y, direction)
        {
            this.time = time;
        }

        /// <summary>
        /// A generator yielding new adjacent TimedMoves. Reimplemented to avoid creating temporary Moves.
        /// </summary>
        /// <returns></returns>
        public new IEnumerable<TimedMove> GetNextMoves()
        {
            Direction[] directions;
            if (Constants.ALLOW_DIAGONAL_MOVE)
                directions = Move.validDirections;
            else
                directions = Move.validDirectionsNoDiag;
            foreach (Direction op in directions)
            {
                yield return new TimedMove(this.x + Move.directionToDeltas[(int)op, 0],
                                           this.y + Move.directionToDeltas[(int)op, 1], op, this.time + 1);
            }
        }

        /// <summary>
        /// Change coordinates in specified direction and increment timestep.
        /// </summary>
        /// <param name="direction"></param>
        public override void Update(Direction direction)
        {
            base.Update(direction);
            this.time += 1;
        }

        public TimedMove(Move cpy, int time)
            : base(cpy)
        {
            this.time = time;
        }

        public TimedMove() { }

        public TimedMove(TimedMove cpy) : base(cpy)
        {
            this.time = cpy.time;
        }

        public override bool Equals(object obj)
        {
            if (this.time != ((TimedMove)obj).time)
                return false;

            //return base.Equals(obj);

            // Begin copied code of base to avoid a method call
            Move that = (Move)obj;
            return (this.x == that.x && this.y == that.y &&
                    ((this.direction == Direction.NO_DIRECTION) || (that.direction == Direction.NO_DIRECTION) ||
                     (this.direction == that.direction)));
            // End copied code of base
        }

        public override int GetHashCode()
        {
            unchecked
            {
                //return base.GetHashCode() * 3 + this.time;

                // Begin copied code of base to avoid a method call:
                int hash = 17;
                hash = 23 * hash + x;
                hash = 23 * hash + y;
                // End copied code of base
                return hash * 3 + this.time;
            }
        }

        public new TimedMove GetMoveWithoutDirection()
        {
            TimedMove copy = new TimedMove(this);
            copy.direction = Direction.NO_DIRECTION;
            return copy;
        }

        /// <summary>
        /// Check if the given move collides with this move.
        /// This includes:
        /// 0. Same time
        /// 1. Head on collision
        /// 2. When other moves target the same location.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool IsColliding(TimedMove other)
        {
            return IsColliding(other.x, other.y, other.direction, other.time);
        }

        public bool IsColliding(int other_x, int other_y, Direction other_direction, int time)
        {
            if (this.time != time)
                return false;

            return base.IsColliding(other_x, other_y, other_direction);
        }

        /// <summary>
        /// Reimplemented to avoid creating temporary Move objects
        /// </summary>
        /// <returns></returns>
        public new TimedMove GetOppositeMove()
        {
            if (direction == Direction.Wait || direction == Direction.NO_DIRECTION)
                return this;
            return new TimedMove(this.x + Move.directionToOppositeDeltas[(int)direction, 0],
                            this.y + directionToOppositeDeltas[(int)direction, 1],
                            directionToOppositeDirection[(int)direction], this.time);
        }

        /// <summary>
        /// Isn't used anywhere
        /// </summary>
        /// <param name="cpy"></param>
        /// <param name="time"></param>
        public void setup(Move cpy, int time)
        {
            base.setup(cpy);
            this.time = time;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="cpy"></param>
        public void setup(TimedMove cpy)
        {
            base.setup(cpy);
            this.time = cpy.time;
        }

        /// <summary>
        /// Almost isn't used anywhere
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="direction"></param>
        /// <param name="time"></param>
        public void setup(int x, int y, Move.Direction direction, int time)
        {
            base.setup(x, y, direction);
            this.time = time;
        }

        public bool IsColliding(ICollection<TimedMove> moves)
        {
            Move.Direction saveDirection = this.direction;
            this.direction = Move.Direction.NO_DIRECTION;
            if (moves.Contains(this))
            {
                this.direction = saveDirection;
                return true;
            }
            this.direction = saveDirection;

            this.setOppositeMove();
            if (moves.Contains(this)) // Check direction too now
            {
                this.setOppositeMove();
                return true;
            }
            this.setOppositeMove();

            return false;
        }

        public bool IsColliding(IReadOnlyDictionary<TimedMove, int> timedMovesToAgentID)
        {
            Move.Direction saveDirection = this.direction;
            this.direction = Move.Direction.NO_DIRECTION;
            if (timedMovesToAgentID.ContainsKey(this))
            {
                this.direction = saveDirection;
                return true;
            }
            this.direction = saveDirection;

            this.setOppositeMove();
            if (timedMovesToAgentID.ContainsKey(this)) // Check direction too now
            {
                this.setOppositeMove();
                return true;
            }
            this.setOppositeMove();

            return false;
        }

        /// <summary>
        /// Gets a dictionary mapping TimedMoves to the agent that already made them
        /// and returns a list of agents this TimedMove collides with.
        /// </summary>
        /// <param name="timedMovesToAgentIndex"></param>
        /// <returns></returns>
        public List<int> GetColliding(IReadOnlyDictionary<TimedMove, int> timedMovesToAgentIndex)
        {
            List<int> ans = null;
            Move.Direction saveDirection = this.direction;
            Direction[] directions;
            if (Constants.ALLOW_DIAGONAL_MOVE)
                directions = Move.validDirections;
            else
                directions = Move.validDirectionsNoDiag;
            foreach (var direction in directions) // TEMP FIX! Need to get rid of the whole NO_DIRECTION SHTICK! It breaks transitivity!
            {
                this.direction = direction;
                if (timedMovesToAgentIndex.ContainsKey(this))
                {
                    if (ans == null)
                        ans = new List<int>(4);
                    ans.Add(timedMovesToAgentIndex[this]);
                }
            }
            this.direction = saveDirection;

            this.setOppositeMove();
            if (timedMovesToAgentIndex.ContainsKey(this)) // Check direction too now
            {
                if (ans == null)
                    ans = new List<int>(1);
                ans.Add(timedMovesToAgentIndex[this]);
            }
            this.setOppositeMove();

            if (ans != null)
                return ans;
            else
                return TimedMove.emptyList;
        }

        private static readonly List<int> emptyList = new List<int>(0);

        /// <summary>
        /// Gets a dictionary mapping TimedMoves to the agents that already made them
        /// and returns a list of agents this TimedMove collides with.
        /// </summary>
        /// <param name="timedMovesToAgentNumLists"></param>
        /// <returns></returns>
        public List<int> GetColliding(IReadOnlyDictionary<TimedMove, List<int>> timedMovesToAgentNumLists)
        {
            List<int> ans = null;
            Move.Direction saveDirection = this.direction;
            
            Direction[] directions;
            if (Constants.ALLOW_DIAGONAL_MOVE)
                directions = Move.validDirections;
            else
                directions = Move.validDirectionsNoDiag;
            
                foreach (var direction in directions) // TEMP FIX! Need to get rid of the whole NO_DIRECTION SHTICK! It breaks transitivity!
                {
                    this.direction = direction;
                    if (timedMovesToAgentNumLists.ContainsKey(this))
                    {
                        if (ans == null)
                            ans = new List<int>(timedMovesToAgentNumLists[this].Count + 3);
                        ans.AddRange(timedMovesToAgentNumLists[this]);
                    }
                }

                this.direction = saveDirection;

                this.setOppositeMove();
                if (timedMovesToAgentNumLists.ContainsKey(this)) // Check direction too now
                {
                    if (ans == null)
                        ans = new List<int>(timedMovesToAgentNumLists[this].Count);
                    ans.AddRange(timedMovesToAgentNumLists[this]);
                }
                this.setOppositeMove();

            if (ans != null)
                return ans;
            else
                return TimedMove.emptyList;
        }
        
        public void UpdateConflictCounts(IReadOnlyDictionary<TimedMove, List<int>> conflictAvoidance,
                                         Dictionary<int, int> conflictCounts, Dictionary<int, List<int>> conflictTimes, Dictionary<int, List<int>> conflictTimesBias, Dictionary<int, List<double>> conflictProbability, int conflictRange = 0, int timeWithoutDelays = -1)
        {
            //if(timeWithoutDelays == -1)
             //   Console.WriteLine("Problem!!!!!!!!!!");
            double delayProbability = updateCollisionProbability();
            int saveTime = this.time;
            int maxPlanFromCA = Math.Max(getMaxPlanFromConflictAvoidance(conflictAvoidance), saveTime);
            if (conflictRange >= 0)
                for (int biasTime = saveTime - conflictRange; biasTime < saveTime + conflictRange + 1; biasTime++)
                {
                    this.time = biasTime;
                    List<int> colliding = this.GetColliding(conflictAvoidance);
                    List<int> visited = new List<int>();
                    foreach (int agentNum in colliding)
                    {
                        if (visited.Contains(agentNum))
                            continue;
                        visited.Add(agentNum);

                        if (conflictCounts.ContainsKey(agentNum) == false)
                            conflictCounts[agentNum] = 0;
                        conflictCounts[agentNum] += 1;
                        if (conflictTimes.ContainsKey(agentNum) == false)
                            conflictTimes[agentNum] = new List<int>(4);
                        if (conflictTimesBias.ContainsKey(agentNum) == false)
                        {
                            conflictTimesBias[agentNum] = new List<int>(4);
                            conflictProbability[agentNum] = new List<double>(4);
                        }
                        if (!conflictTimes[agentNum].Contains(saveTime)  ||
                            (conflictTimes[agentNum].Contains(saveTime) && !conflictTimesBias[agentNum].Contains(this.time - saveTime)))
                        {
                            conflictTimes[agentNum].Add(saveTime);
                            conflictTimesBias[agentNum].Add(this.time - saveTime);
                            conflictProbability[agentNum].Add(1);
                        }
                        else if (conflictTimes[agentNum].Contains(saveTime))
                        {
                            bool contain = false;
                            for (int i = 0; i < conflictTimes[agentNum].Count; i++)
                            {
                                int currentBias = conflictTimesBias[agentNum][i];
                                if (conflictTimes[agentNum][i] == saveTime && conflictTimesBias[agentNum][i] == (this.time - saveTime))
                                    contain = true;
                            }
                            if (contain == false)
                            {
                                conflictTimes[agentNum].Add(saveTime);
                                conflictTimesBias[agentNum].Add(this.time - saveTime);
                                conflictProbability[agentNum].Add(0);
                            }
                        }
                    }
                }
            else if (conflictRange == -1)
            {
               // Dictionary<int, Dictionary<int, TimedMove>> newDic = conflictAvoidanceToDictionary(conflictAvoidance, maxPlanFromCA);
                for (int biasTime = 0/*- CbsNode.maxPlanSizeForPRobust*/; biasTime < maxPlanFromCA * 4; biasTime++)
                {
                    this.time = biasTime;
                    List<int> colliding = this.GetColliding(conflictAvoidance);
                    List<int> visited = new List<int>();
                    foreach (int agentNum in colliding)
                    {
                        if (visited.Contains(agentNum))
                            continue;
                        visited.Add(agentNum);

                        if (conflictCounts.ContainsKey(agentNum) == false)
                            conflictCounts[agentNum] = 0;
                        conflictCounts[agentNum] += 1;
                        if (conflictTimes.ContainsKey(agentNum) == false)
                            conflictTimes[agentNum] = new List<int>(4);
                        if (conflictTimesBias.ContainsKey(agentNum) == false)
                        {
                            conflictTimesBias[agentNum] = new List<int>(4);
                            conflictProbability[agentNum] = new List<double>(4);
                        }
                        if (!conflictTimes[agentNum].Contains(saveTime) ||
                            (conflictTimes[agentNum].Contains(saveTime) && !conflictTimesBias[agentNum].Contains(this.time - saveTime)))
                        {
                            conflictTimes[agentNum].Add(saveTime);
                            conflictTimesBias[agentNum].Add(this.time - saveTime);
                            conflictProbability[agentNum].Add(0);
                        }
                        else if(conflictTimes[agentNum].Contains(saveTime))
                        {
                            bool contain = false;
                            for(int i = 0; i < conflictTimes[agentNum].Count; i++)
                            {
                                int currentBias = conflictTimesBias[agentNum][i];
                                if (conflictTimes[agentNum][i] == saveTime && conflictTimesBias[agentNum][i] == (this.time - saveTime))
                                    contain = true;
                            }
                            if (contain == false)
                            {
                                conflictTimes[agentNum].Add(saveTime);
                                conflictTimesBias[agentNum].Add(this.time - saveTime);
                                conflictProbability[agentNum].Add(0);
                            }
                        }
                            //int moves = getMovesCount(newDic[agentNum], this.time);
                            //double collisionProbability;
                            //Console.WriteLine(moves + " " + timeWithoutDelays);
                            //if (moves != this.time || timeWithoutDelays != saveTime)
                            //    Console.WriteLine("Wait wait wait!");
                            //if (timeWithoutDelays == -1)
                            //    timeWithoutDelays = moves;
                            /* just old
                            if (this.time - saveTime > 0)
                                collisionProbability = calculateCollisionProbability(delayProbability, this.time - saveTime, timeWithoutDelays, moves);
                            else
                                collisionProbability = calculateCollisionProbability(delayProbability, saveTime - this.time, moves, timeWithoutDelays);
                            */
                            /* old old
                            if (moves - timeWithoutDelays > 0)
                                collisionProbability = calculateCollisionProbability(delayProbability, moves - timeWithoutDelays, timeWithoutDelays);
                            else
                                collisionProbability = calculateCollisionProbability(delayProbability, timeWithoutDelays - moves, moves);
                            */
                            // conflictProbability[agentNum].Add(collisionProbability); just old
                        
                        else
                        {
                            Console.WriteLine("what?");
                        }
                    }
                }
            }
            this.time = saveTime;
        }

        private int getMovesCount(Dictionary<int, TimedMove> newDic, int time)
        {
            int moves = 0;
            for (int i = 0; i <= time; i++)
                if (newDic[i].direction != Direction.NO_DIRECTION &&
                    newDic[i].direction != Direction.Wait)
                    moves++;
            return moves;
        }


        private int getMaxPlanFromConflictAvoidance(IReadOnlyDictionary<TimedMove, List<int>> conflictAvoidance)
        {
            int max = 0;
            if (conflictAvoidance.Keys == null)
                return 0;
            foreach (KeyValuePair<TimedMove, List<int>> t in conflictAvoidance)
            { 
                if (max < t.Key.time)
                    max = t.Key.time;
            }
            return max + 1;
        }

        private Dictionary<int,Dictionary<int, TimedMove>> conflictAvoidanceToDictionary(IReadOnlyDictionary<TimedMove, List<int>> conflictAvoidance, int maxPlan)
        {
            int count = 0;
            Dictionary<int, Dictionary<int, TimedMove>> newDic = new Dictionary<int, Dictionary<int, TimedMove>>();
            foreach (KeyValuePair<TimedMove, List<int>> t in conflictAvoidance)
            {
                count++;
            }
            //if (count > 29)
            //    Console.WriteLine("");
            if (conflictAvoidance.Keys == null)
                return newDic;
            foreach (KeyValuePair<TimedMove, List<int>> t in conflictAvoidance)
            {
                foreach (int agentIndex in t.Value)
                {
                    if (!newDic.Keys.Contains(agentIndex))
                        newDic[agentIndex] = new Dictionary<int, TimedMove>();
                    if (!newDic[agentIndex].Keys.Contains(t.Key.time))
                        newDic[agentIndex].Add(t.Key.time, t.Key);
                }
            }
            foreach(int agent in newDic.Keys)
            {
                int currentMax = 0;
                TimedMove lastMove = null;
                foreach(int time in newDic[agent].Keys)
                {
                    if (currentMax <= time)
                    {
                        currentMax  = time;
                        lastMove    = newDic[agent][time];
                    }
                }
                for (int i = currentMax + 1; i < maxPlan; i++)
                    newDic[agent].Add(i, new TimedMove(lastMove));
            }
            return newDic;
        }


        public static double conflictProbability = 1; // delay probability!!!!
        /// <summary>
        /// calculate the probability of collision
        /// </summary>
        public static double updateCollisionProbability()
        {
            return conflictProbability;
        }


        /// <summary>
        /// calculate the probability of collision
        /// </summary>
        public static double calculateCollisionProbability(double delayProbability, int k, int closeAgentDistance, int farAgentDistance = -1, bool MCPexecution = false, double delayProbability2 = -1)
        {
           
            //Console.WriteLine("Calculate: k = " + k + " closeAgentDistance = " + closeAgentDistance);
            if (delayProbability2 == -1)
                delayProbability2 = delayProbability;
            double p = delayProbability;
            double p2 = delayProbability2;
            int n1 = closeAgentDistance;
            int n2 = farAgentDistance;
            double sol = 0;
            double epsilon = 0.00000000000000000000001;
            double gap = Double.PositiveInfinity;
            int m = 0;      //delays
            do
            { 
                double firstAgentMovementProb       = (Math.Pow(1 - p, n1));
                double firstAgentDelayProb          = (Math.Pow(p, k + m));
                double secondAgentMovementProb      = (Math.Pow(1 - p2, n2));
                double secondAgentDelayProb         = (Math.Pow(p2, m));
                double firstAgentMovementCount      = (ChooseFormula(n1 + m + k, m + k));
                double secondAgentMovementCount     = (ChooseFormula(n2 + m, m));
                double firstAgentMovementDoubles    = (ChooseFormula(n1 + m + k - 1, m + k - 1));
                double secondAgentMovementDoubles   = (ChooseFormula(n2 + m - 1, m - 1));
                //Console.WriteLine(firstAgentMovementProb + " " + firstAgentDelayProb + " " + secondAgentMovementProb + " " + secondAgentDelayProb + " " + firstAgentMovementCount + " " + secondAgentMovementCount + " " + firstAgentMovementDoubles + " " + secondAgentMovementDoubles);
                gap = firstAgentMovementProb    *
                        firstAgentDelayProb     *
                        secondAgentMovementProb *
                        secondAgentDelayProb    *
                        (firstAgentMovementCount * secondAgentMovementCount - firstAgentMovementDoubles * secondAgentMovementDoubles);
                sol += gap;
                m++;
            } while (epsilon < (gap / sol));
            return sol;
        }

        private static double ChooseFormula(int N, int K)  //N choose K
        {
            if (N < 0 || K < 0)
                return 0;
            double result = 1;
            for (int i = 1; i <= K; i++)
            {
                result *= N - (K - i);
                result /= i;
            }
            return result;
        }

    }
}
